"""Turning on the spot, on top of the Programmed_Gait_V2 gait generator. Pure python (no Webots), so it can be tested offline.

TurnController sits between whatever produces the commands (a script, a keyboard, a joystick) and GaitGenerator.set_twist():
  turn(rate)      hold a turn. rate is a fraction of omega_max in [-1, 1], positive = to the left. turn(0) is a stop.
  go_to(heading)  turn until the heading (rad, world frame) is reached, then stop by itself
  stop()          halt the rotation

The body only moves in the push stages, each one turning it omega/4 (omega is in rad per gait cycle), so
    yaw rate = omega / gait_period
and that is the whole speed model: see turn_params.json and README.md for what changes it.

Stopping is immediate: stop() makes the stride zero, so the rotation halts in the same step. The legs then finish the stage
they are in and every swing lands at the centre, so after one gait cycle all feet are centred and `idle` becomes True. The
caller should then stop calling GaitGenerator.step() (the robot holds its pose) until a new command clears `idle`.

Two things keep the joints from moving faster than in a steady turn. A swing travels |push total| + (|omega next| - |omega
before|) * foot lever / 2, so a turn that speeds up faster than the feet can follow makes some foot swing too far: speeding
up is ramped (ramp_cycles), slowing down and stopping are not. And a command to turn the other way while the feet are still
set up for the first one would make a foot swing twice as far (up to 2x the joint speed): the rotation stops at once, the
feet centre, and the new direction starts after that (up to one gait cycle later).
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "Programmed_Gait_V2"))

import numpy as np  # noqa: E402

from gait_config import DEFAULTS, load_params  # noqa: E402
from gait_core import GaitGenerator, peak_joint_speed  # noqa: E402

TURN_PARAMS_FILE = os.path.join(HERE, "turn_params.json")
TURN_KEYS = ("omega_max", "ramp_cycles", "kp", "omega_min", "tol_deg")  # settings of TurnController; the other keys are gait parameters


def wrap(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def load_turn_params(source=None):
    """(gait params, turn settings).

    The gait params are the walking ones (gait_params.json, as chosen by GAIT_PARAMS) with the gait keys of the turn file
    laid over them. finish_stages is always on: with it off, a swing starts from the nominal rear end of the stride instead of
    from where the foot is, which makes the joints jump when the stride changes.
    source: path to a JSON file. Default: the TURN_PARAMS environment variable, then turn_params.json next to this file.
    """
    source = source or os.environ.get("TURN_PARAMS") or TURN_PARAMS_FILE
    with open(source) as f:
        cfg = json.load(f)
    unknown = set(cfg) - set(DEFAULTS) - set(TURN_KEYS)
    if unknown:
        raise KeyError(f"Unknown turn parameter(s) in {source}: {sorted(unknown)}")
    missing = set(TURN_KEYS) - set(cfg)
    if missing:
        raise KeyError(f"Missing turn parameter(s) in {source}: {sorted(missing)}")
    gait = load_params()
    gait.update({k: v for k, v in cfg.items() if k in DEFAULTS})
    gait["finish_stages"] = True
    return gait, {k: cfg[k] for k in TURN_KEYS}


def analyse_turn(params, settings, cycles=3.0, dt=0.01):
    """Offline check of a turn speed setting, without any robot: runs a full speed turn from rest in each direction.

    params, settings: as returned by load_turn_params(). Returns the yaw rate at full command (deg/s), the fastest femur or
    tibia target movement over 50 ms (deg/s) and the largest shoulder angle (deg). Raises GaitInfeasible if a foot goes out
    of reach."""
    speed = shoulder = 0.0
    for rate in (1.0, -1.0):
        gen = GaitGenerator(params, dt)
        turner = TurnController(gen, settings["omega_max"], settings["ramp_cycles"])
        turner.turn(rate)
        steps = int(round(cycles * params["gait_period"] / dt))
        joints = np.empty((steps + 1, 8))
        joints[0] = gen.targets
        for i in range(steps):
            previous_stage = gen.stage
            gen.step()
            turner.update(0.0, gen.push_just_finished(previous_stage))
            joints[i + 1] = gen.targets
            shoulder = max(shoulder, float(np.degrees(np.abs(gen.shoulder_targets)).max()))
        speed = max(speed, peak_joint_speed(np.degrees(joints), dt))
    return {"yaw_rate_deg_s": math.degrees(settings["omega_max"]) / params["gait_period"],
            "max_joint_speed_deg_s": speed, "max_shoulder_deg": shoulder}


class TurnController:
    """Call update() once per control step. Not thread safe; the commands are meant to come from the control loop."""

    def __init__(self, gen, omega_max, ramp_cycles=2.0, kp=2.0, omega_min=0.04, tol=math.radians(1.0)):
        """gen: a GaitGenerator with finish_stages on.
        omega_max: fastest turn, rad per gait cycle.
        ramp_cycles: gait cycles the turn takes to speed up from rest to omega_max (a lower speed is reached sooner).
        kp, omega_min, tol: go_to(). The turn is kp*error (rad per cycle, error in rad) limited to omega_max and at least
        omega_min, and the robot stops when the error is below tol (rad). Each push stage removes kp/4 of the error."""
        if not gen.p["finish_stages"]:
            raise ValueError("turning needs finish_stages=True, so that each swing starts from where the foot is")
        self.gen = gen
        gen.stand()
        self.omega_max, self.kp, self.omega_min, self.tol = omega_max, kp, omega_min, tol
        self.ramp_cycles = ramp_cycles
        self.mode = "stop"      # "stop", "rate" or "target"
        self.omega = 0.0        # turn being commanded, rad per cycle
        self._wanted = 0.0      # turn asked for; omega follows it, speeding up no faster than the ramp
        self.target = None      # heading of go_to(), rad
        self.idle = True        # stopped and centred: the gait does not need to run
        self._stop_t = 0.0
        self._feet_dir = 0      # direction (+1 left, -1 right) the feet are set up for, 0 once they are centred
        self._pending = None    # (method, argument) of a command that waits for the feet to centre
        self._apply(0.0)

    @property
    def max_rate_deg_s(self):
        """Yaw rate at full command, deg/s."""
        return math.degrees(self.omega_max) / self.gen.p["gait_period"]

    @property
    def rate_deg_s(self):
        """Yaw rate being commanded now, deg/s (positive = to the left)."""
        return math.degrees(self.omega) / self.gen.p["gait_period"]

    def _apply(self, wanted):
        """Ask for a turn of `wanted` rad per cycle. Slowing down and stopping take effect now, speeding up in update()."""
        self._wanted = wanted
        if abs(wanted) < abs(self.omega) or wanted == 0.0:
            self._set(wanted)

    def _set(self, omega):
        self.omega = omega
        if omega != 0.0:
            self._feet_dir = 1 if omega > 0 else -1
        self.gen.set_twist(omega, forward=0.0)

    def _halt(self):
        if self.mode != "stop":
            self.mode = "stop"
            self._stop_t = self.gen.t
        self._apply(0.0)

    def _after_centring(self, command, argument):
        """Stop now and run command(argument) once the feet are centred."""
        self._pending = (command, argument)
        self._halt()

    def turn(self, rate):
        """Hold a turn at rate (fraction of omega_max, positive = left). Call it as often as you like; 0 is a stop."""
        rate = max(-1.0, min(1.0, rate))
        if rate == 0.0:
            self.stop()
        elif self._feet_dir * rate < 0:
            self._after_centring(self.turn, rate)
        else:
            self._pending = None
            self.mode = "rate"
            self.idle = False
            self._apply(rate * self.omega_max)

    def go_to(self, heading):
        """Turn the shortest way to heading (rad, world frame) and stop there."""
        self._pending = None
        self.mode = "target"
        self.target = heading
        self.idle = False

    def stop(self):
        """Halt the rotation now (and drop a command that was waiting for the feet to centre)."""
        self._pending = None
        self._halt()

    def update(self, yaw, push_just_finished=False):
        """yaw: current heading, rad. push_just_finished: GaitGenerator.push_just_finished() of the step just taken."""
        if abs(self._wanted) > abs(self.omega):  # speeding up
            step = self.omega_max / self.ramp_cycles * self.gen.dt / self.gen.p["gait_period"]
            self._set(math.copysign(min(abs(self._wanted), abs(self.omega) + step), self._wanted))
        if self.mode == "target":
            error = wrap(self.target - yaw)
            if abs(error) < self.tol:
                self.stop()
            else:
                omega = max(-self.omega_max, min(self.omega_max, self.kp * error))
                if abs(omega) < self.omega_min:
                    omega = math.copysign(self.omega_min, error)
                if self._feet_dir * omega < 0:
                    self._after_centring(self.go_to, self.target)
                else:
                    self._apply(omega)
        # Once every foot has had a swing with no stride they are all centred. Freeze at the end of a push stage, when all
        # four feet are on the floor.
        if self.mode == "stop" and not self.idle and push_just_finished \
                and self.gen.t >= self._stop_t + self.gen.p["gait_period"]:
            self.idle = True
            self._feet_dir = 0
            if self._pending:
                command, argument = self._pending
                command(argument)
