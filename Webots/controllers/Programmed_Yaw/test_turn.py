"""Offline checks of TurnController on the Programmed_Gait_V2 generator. No Webots needed.

The robot is replaced by a model that turns omega/4 in every push stage (the kinematics of the gait; the Webots runs agree
with it to a few percent), so this checks the control logic: go_to arrives and stops, stop leaves the feet centred and
frozen, and commands changed at any moment (reversals included) never make the joints move faster than a steady turn does.
Usage: python test_turn.py
"""
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "Programmed_Gait_V2"))

from gait_core import GaitGenerator, STAGES, peak_joint_speed  # noqa: E402
from turn_core import TurnController, analyse_turn, load_turn_params, wrap  # noqa: E402

DT = 0.01


class Rig:
    """A generator, a TurnController and the yaw model."""

    def __init__(self, **gait_overrides):
        params, settings = load_turn_params()
        params.update(gait_overrides)
        self.gen = GaitGenerator(params, DT)
        self.turner = TurnController(self.gen, settings["omega_max"], settings["ramp_cycles"], settings["kp"],
                                     settings["omega_min"], math.radians(settings["tol_deg"]))
        self.yaw = 0.0
        self.targets = [self.gen.targets.copy()]

    def step(self):
        previous = self.gen.stage
        if not self.turner.idle:
            self.gen.step()
            if STAGES[self.gen.stage][0] == "push":  # only the push stages move the body
                self.yaw += self.turner.omega / 4 * DT / self.gen.push_time
        self.turner.update(self.yaw, self.gen.push_just_finished(previous))
        self.targets.append(self.gen.targets.copy())

    def run(self, seconds):
        for _ in range(int(round(seconds / DT))):
            self.step()

    def run_until_idle(self, limit=120.0):
        for _ in range(int(limit / DT)):
            if self.turner.idle:
                return True
            self.step()
        return False

    def peak_speed(self):
        return peak_joint_speed(np.degrees(np.array(self.targets)), DT)


def test_hold_turn_rate():
    rig = Rig()
    period = rig.gen.p["gait_period"]
    rig.turner.turn(1.0)
    rig.run(rig.turner.ramp_cycles * period + period)  # speed up first
    start = rig.yaw
    rig.run(8 * period)
    rate = math.degrees(rig.yaw - start) / (8 * period)
    assert abs(rate - rig.turner.max_rate_deg_s) < 0.05 * rig.turner.max_rate_deg_s, rate
    rig.turner.turn(-0.5)
    before = rig.yaw
    rig.run(4 * rig.gen.p["gait_period"])
    assert (rig.yaw - before) < 0, "turn(-0.5) did not turn to the right"


def test_go_to_arrives_and_freezes():
    for degrees in (5, 45, 90, -60, 170):
        rig = Rig()
        rig.turner.go_to(math.radians(degrees))
        assert rig.run_until_idle(), f"go_to({degrees}) never went idle"
        error = math.degrees(wrap(math.radians(degrees) - rig.yaw))
        assert abs(error) < 1.5, f"go_to({degrees}) ended {error:.2f} deg off"
        assert np.all(rig.gen.D1 == 0) and np.all(rig.gen.lat == 0), "feet are not centred when idle"
        frozen = rig.gen.targets.copy()
        rig.run(2.0)
        assert np.array_equal(frozen, rig.gen.targets), "the pose changed while idle"


def test_stop_halts_at_once():
    rig = Rig()
    rig.turner.turn(1.0)
    rig.run(7.3)  # not at a stage boundary
    before = rig.yaw
    rig.turner.stop()
    rig.run(0.5 * rig.gen.p["gait_period"])
    assert abs(rig.yaw - before) < 1e-9, "the model kept turning after stop()"
    assert rig.run_until_idle(), "never went idle after stop()"


def reference_peak():
    """Fastest joint movement of a full speed turn from rest in either direction, in deg/s."""
    params, settings = load_turn_params()
    return analyse_turn(params, settings)["max_joint_speed_deg_s"]


def test_reversing_waits_for_the_feet_to_centre():
    limit = reference_peak() * 1.1
    period = Rig().gen.p["gait_period"]
    for first, second in ((1.0, -1.0), (-1.0, 1.0)):
        for flip in np.arange(0.0, period, 0.11):  # every moment of the cycle
            rig = Rig()
            rig.turner.turn(first)
            rig.run(3 * period + flip)
            rig.turner.turn(second)
            assert rig.turner.omega == 0.0, "the rotation did not stop at once on a reversal"
            before = rig.yaw
            rig.run(3 * period)
            assert rig.peak_speed() <= limit, f"flip {flip:.2f} s: joints reached {rig.peak_speed():.0f} deg/s, limit {limit:.0f}"
            assert (rig.yaw - before) * second > 0, "never turned the new way"


def test_commands_at_any_moment_do_not_make_the_joints_jump():
    limit = reference_peak() * 1.1

    rng = random.Random(1)
    for trial in range(5):
        rig = Rig()
        for _ in range(60):
            choice = rng.choice(("turn", "turn", "stop", "go_to"))
            if choice == "turn":
                rig.turner.turn(rng.uniform(-1, 1))
            elif choice == "stop":
                rig.turner.stop()
            else:
                rig.turner.go_to(rng.uniform(-math.pi, math.pi))
            rig.run(rng.uniform(0.02, 1.5))
        assert rig.peak_speed() <= limit, f"trial {trial}: joints reached {rig.peak_speed():.0f} deg/s, steady turn {limit / 1.1:.0f}"


def test_needs_finish_stages():
    params, _ = load_turn_params()
    params["finish_stages"] = False
    try:
        TurnController(GaitGenerator(params, DT), 0.3)
    except ValueError:
        return
    raise AssertionError("finish_stages=False was accepted")


def main():
    tests = [test_hold_turn_rate, test_go_to_arrives_and_freezes, test_stop_halts_at_once,
             test_reversing_waits_for_the_feet_to_centre, test_commands_at_any_moment_do_not_make_the_joints_jump,
             test_needs_finish_stages]
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
