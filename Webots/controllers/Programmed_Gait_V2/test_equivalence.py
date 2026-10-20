"""Checks that Programmed_Gait_V2 at its default parameters commands the same joint targets as the original Programmed_Gait.

The original controller is run against a stub of the Webots API and every setPosition call is recorded.
Usage: python test_equivalence.py        (no Webots needed)
"""
import contextlib
import io
import os
import runpy
import sys
import tempfile
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ORIGINAL = os.path.join(HERE, "..", "Programmed_Gait", "Programmed_Gait.py")
sys.path.insert(0, HERE)

from gait_config import DEFAULTS  # noqa: E402
from gait_core import GaitGenerator, JOINT_NAMES  # noqa: E402

TIMESTEP = 10  # ms, same as the world


class _Field:
    def setSFVec3f(self, *_): pass
    def setSFRotation(self, *_): pass


class _Node:
    def getField(self, _): return _Field()


def run_original(period, steps):
    """Joint targets (rad) the original controller commands: one row for the start pose, then one per walking step."""
    calls = []

    class Motor:
        def __init__(self, name): self.name = name
        def setPosition(self, value): calls.append((self.name, value))
        def enable(self, *_): pass

    class Supervisor:
        def __init__(self): self.n = 0
        def getSelf(self): return _Node()
        def getFromDef(self, _): return _Node()
        def getDevice(self, name): return Motor(name)
        def step(self, _):
            self.n += 1
            return 0 if self.n <= steps else -1

    stub = types.ModuleType("controller")
    stub.Robot = stub.Accelerometer = object
    stub.Supervisor = Supervisor
    sys.modules["controller"] = stub

    source = open(ORIGINAL).read()
    assert "GAIT_PERIOD = 16" in source
    source = source.replace("GAIT_PERIOD = 16", f"GAIT_PERIOD = {period}")
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "original_under_test.py")
        with open(path, "w") as f:
            f.write(source)
        with contextlib.redirect_stdout(io.StringIO()):
            runpy.run_path(path, run_name="__main__")

    assert len(calls) % 8 == 0
    by_name = {}
    for i in range(0, len(calls), 8):
        assert [c[0] for c in calls[i:i + 8]] == JOINT_NAMES
    return np.array([[c[1] for c in calls[i:i + 8]] for i in range(0, len(calls), 8)])


def run_v2(params, steps):
    dt = TIMESTEP * 1e-3
    gen = GaitGenerator(params, dt)
    rows = [gen.targets.copy()]
    delay = params["init_delay"]
    for _ in range(steps):
        if delay > 0:
            delay -= dt
            continue
        gen.step()
        rows.append(gen.targets.copy())
    return np.array(rows)


def main():
    ok = True
    for period, seconds in ((16, 45), (4, 20)):
        steps = int(seconds / (TIMESTEP * 1e-3))
        ref = run_original(period, steps)
        new = run_v2(dict(DEFAULTS, gait_period=float(period)), steps)
        assert ref.shape == new.shape, (ref.shape, new.shape)
        err = np.abs(ref - new).max()
        print(f"gait_period={period:>2} s: {len(ref) - 1} walking steps, max difference {err:.2e} rad ({np.degrees(err):.2e} deg)")
        ok &= err < 1e-4  # the original computes in float32
    print("OK" if ok else "MISMATCH")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
