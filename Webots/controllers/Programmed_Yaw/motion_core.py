"""Walking forward and turning on the spot as one command, on the Programmed_Gait_V2 gait generator. Pure python (no Webots).

MotionController.drive(forward, turn) takes what a teleoperator holds down: forward in [0, 1] (a fraction of the full stride) and
turn in [-1, 1] (positive = to the left), and stop() halts. Three things can happen:
  forward > 0         walk. Without a turn the heading is held (the same PI loop as Programmed_Gait_V2, so the robot walks
                      straight); with one it walks an arc, and when the turn is let go it holds the heading it ended on.
  forward = 0, turn   turn on the spot (TurnController).
  both 0              stop.

Walking and turning on the spot use different gait parameters: a turn needs a faster period and a lower foot lift than the
walk can have (see README.md). The generator can only change them while stopped with the feet centred, so going from one to the
other means stopping first and waiting for the legs to re-centre (up to one gait cycle of the gait that was running).

Stopping works as in TurnController: the stride is zeroed at once, the legs finish the stage they are in, every swing lands at the
centre, and after one gait cycle `idle` is True and the caller can stop stepping the gait. Speeding up the stride is ramped, which
keeps a foot from having to swing much further than in a steady walk, and the turn while walking is small for the same reason.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "Programmed_Gait_V2"))

from gait_config import DEFAULTS, load_params  # noqa: E402
from gait_core import YawController  # noqa: E402

WALK_PARAMS_FILE = os.path.join(HERE, "walk_params.json")
WALK_KEYS = ("ramp_cycles", "turn_max")  # settings of WalkController; the other keys are gait parameters


def load_walk_params(source=None):
    """(gait params, walk settings). The gait params are the walking ones, gait_params.json of Programmed_Gait_V2 as chosen by
    GAIT_PARAMS, with the gait keys of the walk file laid over them. finish_stages is always on: a swing has to start from where
    the foot is, or a change of stride makes the joints jump. The walk file is walk_params.json, or the file named by the
    WALK_PARAMS environment variable."""
    source = source or os.environ.get("WALK_PARAMS") or WALK_PARAMS_FILE
    with open(source) as f:
        cfg = json.load(f)
    unknown = set(cfg) - set(DEFAULTS) - set(WALK_KEYS)
    if unknown:
        raise KeyError(f"Unknown walk parameter(s) in {source}: {sorted(unknown)}")
    missing = set(WALK_KEYS) - set(cfg)
    if missing:
        raise KeyError(f"Missing walk parameter(s) in {source}: {sorted(missing)}")
    gait = load_params()
    gait.update({k: v for k, v in cfg.items() if k in DEFAULTS})
    gait["finish_stages"] = True
    return gait, {k: cfg[k] for k in WALK_KEYS}


class WalkController:
    """Walks forward holding the heading. Call update() once per control step."""

    def __init__(self, gen, kp, ki, turn_max=0.07, ramp_cycles=2.0):
        """gen: a GaitGenerator with finish_stages on.
        kp, ki: gains of the heading loop (yaw_kp and yaw_ki of the gait parameters, rad per cycle per rad of error).
        turn_max: largest turn while walking, rad per gait cycle, for the heading loop and for a commanded turn.
        ramp_cycles: gait cycles the stride takes to grow from nothing to full."""
        if not gen.p["finish_stages"]:
            raise ValueError("walking needs finish_stages=True, so that each swing starts from where the foot is")
        self.gen = gen
        self.omega_max = turn_max
        self.ramp_cycles = ramp_cycles
        self.mode = "stop"      # "stop" or "walk"
        self.idle = True        # stopped and centred
        self.forward = 0.0      # stride being commanded, fraction of the full stride
        self.omega = 0.0        # turn being commanded, rad per gait cycle
        self._wanted_forward = 0.0
        self._turn = 0.0        # the turn asked for, fraction of omega_max
        self._hold = YawController(kp, ki, self.omega_max, 0.0)
        self._hold_omega = 0.0
        self._need_reference = False
        self._stop_t = 0.0

    @property
    def rate_deg_s(self):
        """Yaw rate being commanded now, deg/s (positive = to the left)."""
        return math.degrees(self.omega) / self.gen.p["gait_period"]

    def drive(self, forward, turn=0.0):
        """Walk at forward (fraction of the full stride, > 0) turning at turn (fraction of the largest turn)."""
        if forward <= 0.0:
            self.stop()
            return
        if self.mode != "walk":
            self.mode = "walk"
            self.idle = False
            self._need_reference = True
        self._wanted_forward = forward
        self._turn = turn

    def stop(self):
        """Halt now."""
        if self.mode != "stop":
            self.mode = "stop"
            self._stop_t = self.gen.t
        self.forward = self.omega = self._wanted_forward = self._turn = 0.0
        self.gen.set_twist(0.0, forward=0.0)

    def update(self, yaw, push_just_finished=False):
        """yaw: current heading, rad. push_just_finished: GaitGenerator.push_just_finished() of the step just taken."""
        gen = self.gen
        if self.mode == "walk":
            per_step = gen.dt / gen.p["gait_period"]  # part of a gait cycle one control step takes
            if self._wanted_forward > self.forward:  # speeding up is ramped, slowing down is not
                self.forward = min(self._wanted_forward, self.forward + per_step / self.ramp_cycles)
            else:
                self.forward = self._wanted_forward

            if self._turn != 0.0:  # a turn in progress: its own heading is the one to hold when it ends
                wanted = self._turn * self.omega_max
                self._need_reference = True
            else:
                if self._need_reference:
                    self._hold.ref, self._hold.integral, self._hold_omega = yaw, 0.0, 0.0
                    self._need_reference = False
                if push_just_finished:  # the body only moves in the push stages
                    self._hold_omega = self._hold.update(yaw)
                wanted = self._hold_omega
            self.omega = wanted
            gen.set_twist(self.omega, forward=self.forward)
        elif not self.idle and push_just_finished and gen.t >= self._stop_t + gen.p["gait_period"]:
            self.idle = True  # every foot has had a swing with no stride: all centred


def add_walking(gen, turner):
    """The MotionController that adds walking to turner (a TurnController on gen, which holds the turn parameters). The walking
    parameters and settings come from load_walk_params()."""
    walk_params, walk_settings = load_walk_params()
    walker = WalkController(gen, walk_params["yaw_kp"], walk_params["yaw_ki"], walk_settings["turn_max"],
                            walk_settings["ramp_cycles"])
    return MotionController(gen, turner, walker, gen.p, walk_params)


class MotionController:
    """Walks and turns on the spot on one GaitGenerator, switching between the two sets of gait parameters when needed."""

    def __init__(self, gen, turner, walker, turn_params, walk_params):
        """turner: the TurnController of gen, which starts with the turn parameters (turn_params) in gen. walker: the
        WalkController of gen, which uses walk_params."""
        self.gen, self.turner, self.walker = gen, turner, walker
        self._params = {"turn": turn_params, "walk": walk_params}
        self._controller = {"turn": turner, "walk": walker}
        self._active = "turn"
        self._pending = None  # (forward, turn) waiting for the other gait to be possible

    @property
    def active(self):
        return self._controller[self._active]

    @property
    def idle(self):
        """Stopped, centred and nothing waiting: the gait does not need to run."""
        return self.active.idle and self._pending is None

    @property
    def mode(self):
        """"stop", "turn" (on the spot) or "walk"."""
        return "stop" if self.active.mode == "stop" else self._active

    @property
    def forward(self):
        return self.walker.forward if self._active == "walk" else 0.0

    @property
    def rate_deg_s(self):
        return self.active.rate_deg_s

    @property
    def max_rate_deg_s(self):
        """Yaw rate at full command when turning on the spot, deg/s."""
        return self.turner.max_rate_deg_s

    def drive(self, forward, turn=0.0):
        """forward in [0, 1], turn in [-1, 1] (positive = left); (0, 0) is a stop. Call it as often as you like."""
        forward = max(0.0, min(1.0, forward))
        turn = max(-1.0, min(1.0, turn))
        if forward == 0.0 and turn == 0.0:
            self.stop()
            return
        wanted = "walk" if forward > 0.0 else "turn"
        if wanted == self._active:
            self._pending = None
            if wanted == "walk":
                self.walker.drive(forward, turn)
            else:
                self.turner.turn(turn)
        elif self.active.idle:
            self._switch(wanted)
            self.drive(forward, turn)
        else:  # stop what is running, and start the other when the legs are centred
            self._pending = (forward, turn)
            self.active.stop()

    def stop(self):
        """Halt now, and forget a command that was waiting."""
        self._pending = None
        self.active.stop()

    def update(self, yaw, push_just_finished=False):
        """yaw: current heading, rad. push_just_finished: GaitGenerator.push_just_finished() of the step just taken."""
        self.active.update(yaw, push_just_finished)
        if self._pending is not None and self.active.idle:
            forward, turn = self._pending
            self._pending = None
            self.drive(forward, turn)

    def _switch(self, name):
        self.gen.use_params(self._params[name])
        self._active = name
