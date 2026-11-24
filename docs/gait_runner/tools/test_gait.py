#!/usr/bin/env python3
"""Checks the C walk (Src/gait.c) against the Python code it was ported from.

The reference is the Webots code itself: GaitGenerator (Programmed_Gait_V2/gait_core.py) driven by WalkController
(Programmed_Yaw/motion_core.py) with the walk parameters, stepped as in Programmed_Yaw.py, with the heading held at 0
(so no turn is ever commanded, as on the board). Each scenario is a script of commands (dt, drive, stop, step); the
same script is fed to tools/gait_host.c, and every step is compared: stage, idle flag and the 8 joint targets.

    python3 tools/test_gait.py build/host/gait_host
"""
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "Webots", "controllers", "Programmed_Yaw"))
sys.path.insert(0, os.path.join(REPO, "Webots", "controllers", "Programmed_Gait_V2"))

from gait_core import GaitGenerator  # noqa: E402
from motion_core import WalkController, load_walk_params  # noqa: E402

TOL_DEG = 0.01     # float on the board vs float64 in Python


class Reference:
    """The Python walk, recording the command script it was given."""

    def __init__(self, dt):
        self.params, self.settings = load_walk_params()
        self.gen = GaitGenerator(self.params, dt)
        self.gen.stand()
        self.walker = WalkController(self.gen, self.params["yaw_kp"], self.params["yaw_ki"],
                                     self.settings["turn_max"], self.settings["ramp_cycles"])
        self.walker.stop()      # stride 0, as gait_init
        self.script = [f"dt {dt!r}"]
        self.rows = []

    @property
    def period(self):
        return self.params["gait_period"]

    def drive(self, forward):
        self.walker.drive(forward)
        self.script.append(f"drive {forward!r}")

    def stop(self):
        self.walker.stop()
        self.script.append("stop")

    def step(self, n=1):
        for _ in range(n):
            prev = self.gen.stage
            if not self.walker.idle:
                self.gen.step()
            self.walker.update(0.0, self.gen.push_just_finished(prev))
            self.script.append("step")
            self.rows.append((self.gen.stage, int(self.walker.idle), self.gen.t, *self.gen.targets))

    def step_until(self, cond, limit=100000):
        for _ in range(limit):
            if cond():
                return
            self.step()
        raise RuntimeError("scenario did not reach its condition")


def scenario_walk(dt, forward, cycles):
    """Stand, walk (ramp + cycles), stop, wait until idle, then a few idle steps."""
    def make():
        r = Reference(dt)
        r.step(20)
        r.drive(forward)
        t_end = (r.settings["ramp_cycles"] + cycles) * r.period
        r.step_until(lambda: r.gen.t >= t_end)
        r.stop()
        r.step_until(lambda: r.walker.idle)
        r.step(20)
        return r
    return make


def scenario_restart():
    """Stop in the middle of the ramp, wait for idle, stand still, walk again, stop in the middle of a swing."""
    def make():
        r = Reference(0.01)
        r.drive(1.0)
        r.step_until(lambda: r.gen.t >= 1.3 * r.period)
        r.stop()
        r.step_until(lambda: r.walker.idle)
        r.step(30)
        r.drive(0.8)
        r.step_until(lambda: r.gen.t >= 4.55 * r.period)
        r.stop()
        r.step_until(lambda: r.walker.idle)
        r.step(5)
        return r
    return make


SCENARIOS = {
    "full speed, full stride (as in Webots)": scenario_walk(0.01, 1.0, 4),
    "half speed (board default), full stride": scenario_walk(0.005, 1.0, 4),
    "0.3 speed, 60 % stride": scenario_walk(0.003, 0.6, 2),
    "stop in the ramp, restart, stop in a swing": scenario_restart(),
}


def run_c(binary, script):
    out = subprocess.run([binary], input="\n".join(script) + "\n", capture_output=True, text=True, check=True).stdout
    rows = []
    for line in out.splitlines():
        v = line.split()
        rows.append((int(v[0]), int(v[1]), float(v[2]), *[float(x) for x in v[3:11]], int(v[11])))
    return rows


def compare(name, ref, c_rows):
    if len(c_rows) != len(ref.rows):
        print(f"FAIL {name}: {len(c_rows)} C steps, {len(ref.rows)} Python steps")
        return False
    ok = True
    worst = 0.0
    for i, (p, c) in enumerate(zip(ref.rows, c_rows)):
        if c[11] != 1:
            print(f"FAIL {name}: step {i}: C foot out of reach")
            return False
        if p[0] != c[0] or p[1] != c[1]:
            print(f"FAIL {name}: step {i}: stage/idle Python {p[0]}/{p[1]} C {c[0]}/{c[1]}")
            return False
        d = max(abs(math.degrees(a - b)) for a, b in zip(p[3:11], c[3:11]))
        worst = max(worst, d)
    ok = worst <= TOL_DEG
    deg = np.degrees(np.array([r[3:11] for r in ref.rows]))
    dt = ref.gen.dt
    n = max(1, int(round(0.05 / dt)))
    peak_gait = np.abs(deg[n:] - deg[:-n]).max() / (n * dt)  # deg per second of gait time
    print(f"{'ok  ' if ok else 'FAIL'} {name}: {len(c_rows)} steps, largest difference {worst:.2e} deg; "
          f"femur {deg[:, 0::2].min():.1f}..{deg[:, 0::2].max():.1f}, tibia {deg[:, 1::2].min():.1f}..{deg[:, 1::2].max():.1f} deg, "
          f"peak {peak_gait:.0f} deg/s of gait time")
    return ok


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    binary = sys.argv[1]
    ok = True
    for name, make in SCENARIOS.items():
        ref = make()
        ok &= compare(name, ref, run_c(binary, ref.script))
    print("C walk matches the Python walk" if ok else "MISMATCH between the C and the Python walk")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
