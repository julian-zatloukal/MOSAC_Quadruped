"""Offline checks of walking (forward and backwards) and of switching between walking and turning on the spot
(MotionController). No Webots needed.

The robot is replaced by a model that turns (omega + a constant disturbance) / 4 per cycle in every push stage and moves by as
much as the stance feet slide, so this checks the control logic: the stride ramps up and stops at once, the heading is held, the
legs re-centre before the gait changes, and whatever is commanded, however fast, the joints never move much faster than in a
steady walk.
Usage: python test_motion.py
"""
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from motion_core import add_walking, load_walk_params, WalkController  # noqa: E402
from turn_core import TurnController, load_turn_params  # noqa: E402
from gait_core import BL, BR, FL, FR, GaitGenerator, STAGES, peak_joint_speed  # noqa: E402  (turn_core puts Programmed_Gait_V2 on the path)

DT = 0.01


class Rig:
    def __init__(self, disturbance=0.0):
        """disturbance: rad per gait cycle the body turns on its own (a heading error the loop has to remove)."""
        params, settings = load_turn_params()
        self.gen = GaitGenerator(params, DT)
        self.turner = TurnController(self.gen, settings["omega_max"], settings["ramp_cycles"], settings["kp"],
                                     settings["omega_min"], math.radians(settings["tol_deg"]))
        self.motion = add_walking(self.gen, self.turner)
        self.disturbance = disturbance
        self.yaw = 0.0
        self.travel = 0.0  # mm the body moved forward (negative = backwards)
        self.targets = [self.gen.targets.copy()]
        self.swings = []   # legs in the order they swung

    def step(self):
        previous = self.gen.stage
        if not self.motion.idle:
            feet = self.gen.D1.copy()
            self.gen.step()
            if STAGES[self.gen.stage][0] == "push":  # only the push stages move the body
                self.yaw += (self.motion.active.omega + self.disturbance) / 4 * DT / self.gen.push_time
                self.travel += float(np.mean(self.gen.D1 - feet))  # feet sliding back = body moving forward
            if self.gen.stage != previous and STAGES[self.gen.stage][0] == "swing":
                self.swings.append(self.gen._stages[self.gen.stage][1])
        self.motion.update(self.yaw, self.gen.push_just_finished(previous))
        self.targets.append(self.gen.targets.copy())

    def run(self, seconds):
        for _ in range(int(round(seconds / DT))):
            self.step()

    def run_until(self, condition, limit=60.0):
        for _ in range(int(limit / DT)):
            if condition():
                return True
            self.step()
        return False

    @property
    def period(self):
        return self.gen.p["gait_period"]

    def peak_speed(self, start=0):
        return peak_joint_speed(np.degrees(np.array(self.targets[start:])), DT)

    def biggest_step(self):
        """Largest movement of any joint target in one control step, deg."""
        return float(np.abs(np.diff(np.degrees(np.array(self.targets)), axis=0)).max())


def steady_walk_peak(forward=1.0):
    """Fastest joint movement of a steady walk, deg/s."""
    rig = Rig()
    rig.motion.drive(forward, 0.0)
    rig.run(8 * rig.gen.p["gait_period"])
    start = len(rig.targets)
    rig.run(4 * rig.period)
    return rig.peak_speed(start - 50)


def test_walking_ramps_up_and_stops_at_once():
    rig = Rig()
    rig.motion.drive(1.0, 0.0)
    assert rig.motion.mode == "walk" and rig.motion.forward == 0.0
    rig.run(rig.period)
    assert 0.2 < rig.motion.forward < 0.8, "the stride should be growing"
    rig.run(3 * rig.period)
    assert rig.motion.forward == 1.0

    rig.run(1.3)  # not at a stage boundary
    rig.motion.stop()
    assert rig.motion.forward == 0.0 and rig.motion.active.omega == 0.0, "stop must zero the stride at once"
    assert rig.run_until(lambda: rig.motion.idle, limit=2 * rig.period), "never went idle"
    assert np.all(rig.gen.D1 == 0) and np.all(rig.gen.lat == 0), "feet are not centred when idle"
    frozen = rig.gen.targets.copy()
    rig.run(2.0)
    assert np.array_equal(frozen, rig.gen.targets), "the pose changed while idle"


def test_a_fraction_of_the_stride_walks_a_shorter_stride():
    rig = Rig()
    rig.motion.drive(0.5, 0.0)
    rig.run(6 * rig.period)
    assert abs(rig.motion.forward - 0.5) < 1e-9


def test_the_heading_is_held():
    drift = 0.02  # rad per cycle, 1.1 deg
    for forward in (1.0, -1.0):
        rig = Rig(disturbance=drift)
        rig.motion.drive(forward, 0.0)
        rig.run(25 * rig.period)
        assert abs(math.degrees(rig.yaw)) < 1.5, f"forward={forward}: heading drifted to {math.degrees(rig.yaw):.1f} deg"
    # the same disturbance with the loop switched off would have turned the body this much
    assert math.degrees(drift) * 25 > 20


def test_walking_backwards():
    rig = Rig()
    rig.motion.drive(-1.0, 0.0)
    assert rig.motion.mode == "back" and rig.gen.p["backward"] and rig.motion.forward == 0.0
    rig.run(rig.period)
    assert -0.8 < rig.motion.forward < -0.2, "the stride should be growing"
    rig.run(3 * rig.period)
    assert rig.motion.forward == -1.0
    start, swings = rig.travel, len(rig.swings)
    rig.run(4 * rig.period)
    stride = load_walk_params(backward=True)[0]["parabola_length"]
    assert abs((rig.travel - start) / 4 + stride) < 0.03 * stride, f"{(rig.travel - start) / 4:.1f} mm per cycle"
    assert rig.swings[swings:swings + 8] in ([BL, FR, BR, FL] * 2, [FR, BR, FL, BL] * 2, [BR, FL, BL, FR] * 2,
                                             [FL, BL, FR, BR] * 2), rig.swings[swings:]

    rig.motion.drive(-0.5, 0.0)  # a shorter stride
    rig.run(2 * rig.period)
    assert rig.motion.forward == -0.5
    rig.motion.stop()
    assert rig.motion.forward == 0.0
    assert rig.run_until(lambda: rig.motion.idle, limit=2 * rig.period), "never went idle"
    assert np.all(rig.gen.D1 == 0) and np.all(rig.gen.lat == 0), "feet are not centred when idle"


def test_steering_while_walking_backwards():
    rig = Rig()
    rig.motion.drive(-1.0, 1.0)  # yaw left: the nose turns left (counter-clockwise), whichever way the robot walks
    rig.run(8 * rig.period)
    assert math.degrees(rig.yaw) > 5 and rig.travel < 0, (math.degrees(rig.yaw), rig.travel)
    rig.motion.drive(-1.0, 0.0)
    rig.run(0.5)
    held = rig.yaw
    rig.run(8 * rig.period)
    assert abs(math.degrees(rig.yaw - held)) < 1.0, "kept turning after the turn was let go"


def test_steering_while_walking_and_holding_the_new_heading():
    rig = Rig()
    turn_max = load_walk_params()[1]["turn_max"]
    rig.motion.drive(1.0, 1.0)
    rig.run(6 * rig.period)
    start = rig.yaw
    rig.run(5 * rig.period)
    rate = (rig.yaw - start) / (5 * rig.period)  # rad/s
    expected = turn_max / rig.period
    assert abs(rate - expected) < 0.1 * expected, (rate, expected)
    rig.motion.drive(1.0, 0.0)  # let go of the turn: hold the heading it ended on
    rig.run(0.5)
    held = rig.yaw
    rig.run(8 * rig.period)
    assert abs(math.degrees(rig.yaw - held)) < 1.0, "kept turning after the turn was let go"
    rig.motion.drive(1.0, -1.0)
    rig.run(6 * rig.period)
    assert rig.yaw < held, "never turned the other way"


def test_switching_waits_for_the_legs_to_centre():
    rig = Rig()
    turn_period = rig.period
    rig.motion.drive(1.0, 0.0)
    rig.run(5 * turn_period)  # the walking gait has a different period and shares the cycle out between the stages differently
    walk_period = rig.period
    assert walk_period != turn_period and rig.motion.mode == "walk"

    rig.motion.drive(0.0, 1.0)  # turn on the spot: the walk has to stop and centre first
    assert rig.motion.mode == "stop" and rig.motion.forward == 0.0 and not rig.motion.idle
    assert rig.run_until(lambda: rig.motion.mode == "turn", limit=2 * walk_period), "never started turning"
    assert rig.period == turn_period, "the turning parameters were not selected"
    rig.run(4 * turn_period)
    assert rig.motion.rate_deg_s > 0 and rig.yaw > 0

    rig.motion.drive(1.0, 0.0)  # and back to walking
    assert rig.motion.mode == "stop"
    assert rig.run_until(lambda: rig.motion.mode == "walk", limit=2 * turn_period), "never started walking"
    assert rig.period == walk_period
    rig.run(3 * walk_period)
    assert rig.motion.forward > 0.5

    rig.motion.drive(-1.0, 0.0)  # and to walking backwards: another order of the swings
    assert rig.motion.mode == "stop" and not rig.gen.p["backward"]
    assert rig.run_until(lambda: rig.motion.mode == "back", limit=2 * walk_period), "never started walking backwards"
    assert rig.gen.p["backward"]
    rig.run(3 * rig.period)
    assert rig.motion.forward < -0.5
    rig.motion.drive(1.0, 0.0)
    assert rig.run_until(lambda: rig.motion.mode == "walk", limit=2 * rig.period), "never walked forward again"
    assert not rig.gen.p["backward"] and rig.period == walk_period

    rig.motion.drive(0.0, 1.0)  # a waiting command is dropped by a stop
    rig.motion.stop()
    rig.run(3 * walk_period)
    assert rig.motion.mode == "stop" and rig.motion.idle, "a stop must cancel a command that was waiting"


def test_the_joints_do_not_jump_when_switching():
    reference = Rig()
    reference.motion.drive(1.0, 0.0)
    reference.run(10 * reference.period)
    walking = reference.biggest_step()
    reference = Rig()
    reference.motion.drive(-1.0, 0.0)
    reference.run(10 * reference.period)
    walking = max(walking, reference.biggest_step())
    reference = Rig()
    reference.motion.drive(0.0, 1.0)
    reference.run(10 * reference.period)
    limit = 1.25 * max(walking, reference.biggest_step())

    rig = Rig()
    for forward, turn in ((1.0, 0.0), (0.0, 1.0), (1.0, 0.0), (0.0, -1.0), (1.0, 1.0), (0.0, 1.0), (-1.0, 0.0), (1.0, -1.0),
                          (-1.0, 1.0), (0.0, -1.0), (-0.5, 0.0)):
        rig.motion.drive(forward, turn)
        rig.run(12.0)
    assert rig.biggest_step() <= limit, f"a joint target moved {rig.biggest_step():.1f} deg in one step, limit {limit:.1f}"


def test_commands_at_any_moment_keep_the_joints_near_a_steady_walk():
    steady = {forward: steady_walk_peak(forward) for forward in (1.0, -1.0)}
    limit = 1.25 * max(steady.values())
    rng = random.Random(1)
    worst = 0.0
    for trial in range(8):
        rig = Rig()
        for _ in range(30):
            rig.motion.drive(rng.choice((0.0, 1.0, 1.0, 0.5, -1.0, -1.0, -0.5)),
                             rng.choice((-1.0, 0.0, 0.0, 1.0, rng.uniform(-1, 1))))
            rig.run(rng.uniform(0.05, 7.0))
        worst = max(worst, rig.peak_speed())
        assert rig.peak_speed() <= limit, f"trial {trial}: joints reached {rig.peak_speed():.0f} deg/s, limit {limit:.0f}"
    print(f"      worst of the random runs: {worst:.0f} deg/s, steady walk {steady[1.0]:.0f} forward, "
          f"{steady[-1.0]:.0f} backwards")


def test_stage_weights_share_out_the_cycle():
    params, _ = load_walk_params()
    gen = GaitGenerator(params, DT)
    gen.set_twist(0.0)
    period = params["gait_period"]
    seconds = {kind: 0.0 for kind in ("lean", "swing", "restore", "push")}
    steps = int(round(3 * period / DT))
    for _ in range(steps):
        gen.step()
        seconds[STAGES[gen.stage][0]] += DT
    weight = {kind: params["weight_" + kind] for kind in seconds}
    total = sum(weight.values())
    for kind in seconds:  # three cycles, four stages of each kind in a cycle
        expected = 3 * period * weight[kind] / total
        assert abs(seconds[kind] - expected) < 4 * DT, f"{kind} took {seconds[kind]:.2f} s, expected {expected:.2f} s"
    # however long the push stages take, a cycle moves every foot back by one stride
    one = GaitGenerator(params, DT)
    one.set_twist(0.0)
    pushed = np.zeros(4)
    for _ in range(int(round(period / DT))):
        d = one.D1.copy()
        one.step()
        if STAGES[one.stage][0] == "push":
            pushed += one.D1 - d
    assert np.allclose(pushed, params["parabola_length"], rtol=0.02), pushed


def test_guards():
    params, _ = load_turn_params()
    params["finish_stages"] = False
    try:
        WalkController(GaitGenerator(params, DT), 1.2, 0.12)
        raise AssertionError("finish_stages=False was accepted")
    except ValueError:
        pass
    rig = Rig()
    other = dict(rig.gen.p, standing_height=rig.gen.p["standing_height"] + 5)
    try:
        rig.gen.use_params(other)
        raise AssertionError("gait parameters with another standing height were accepted")
    except ValueError:
        pass


def main():
    tests = [test_walking_ramps_up_and_stops_at_once, test_a_fraction_of_the_stride_walks_a_shorter_stride,
             test_the_heading_is_held, test_walking_backwards, test_steering_while_walking_and_holding_the_new_heading,
             test_steering_while_walking_backwards,
             test_switching_waits_for_the_legs_to_centre, test_the_joints_do_not_jump_when_switching,
             test_commands_at_any_moment_keep_the_joints_near_a_steady_walk, test_stage_weights_share_out_the_cycle,
             test_guards]
    failed = 0
    for test in tests:
        try:
            test()
            print(f"ok    {test.__name__}")
        except AssertionError as error:
            failed += 1
            print(f"FAIL  {test.__name__}: {error}")
    print("OK" if not failed else f"{failed} FAILED")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
