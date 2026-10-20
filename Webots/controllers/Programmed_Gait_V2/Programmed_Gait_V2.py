"""Programmed_Gait_V2 controller.

The crawl gait of the original Programmed_Gait controller, with
  - its constants in gait_params.json (tuned with tune_gait.py), defaults = the original constants
  - yaw feedback: the heading error makes the stance feet slide as for a turn (longer stride on the outer side, front feet
    slid sideways against the rear ones by the shoulder joints), so the robot keeps walking straight. The heading is read
    from the simulator (a stand-in for an IMU yaw estimate on the real robot).

Environment variables (only needed by tune_gait.py; to watch the robot just open the world):
  GAIT_PARAMS     parameter file, or "defaults" for the original constants. Default: gait_params.json next to this file
  GAIT_RESULT     if set, run in evaluation mode: walk for a fixed window, write the metrics as JSON to this path, quit
  GAIT_WARMUP_S   evaluation mode: seconds to walk before measuring (rounded up to whole gait cycles, default 6)
  GAIT_MEASURE_S  evaluation mode: seconds measured (rounded up to whole gait cycles, at least 2, default 24)
"""
import json
import math
import os
import time

import numpy as np
from controller import Supervisor

from gait_config import load_params
from gait_core import GaitGenerator, JOINT_NAMES, SHOULDER_NAMES, YawController, peak_joint_speed

FALL_TILT_DEG = 50.0  # same critical angle as Environment.py
FALL_HEIGHT = 0.10    # m, body height below which the robot is considered fallen
RESET_POS = [0.0, 0.0, 0.141]

supervisor = Supervisor()
robot_node = supervisor.getFromDef('MOSAC')
timestep = int(supervisor.getBasicTimeStep())
dt = timestep * 1e-3

robot_node.getField("translation").setSFVec3f(RESET_POS)
robot_node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, 0.0])

motors = [supervisor.getDevice(name) for name in JOINT_NAMES]
shoulders = [supervisor.getDevice(name) for name in SHOULDER_NAMES]
for motor in motors:
    motor.enableTorqueFeedback(timestep)


def wrap(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def read_pose():
    """Position (m) and yaw, roll, pitch (rad) of the body."""
    pos = robot_node.getPosition()
    R = robot_node.getOrientation()
    yaw = math.atan2(R[3], R[0])
    roll = math.atan2(R[7], R[8])
    pitch = -math.asin(max(-1.0, min(1.0, R[6])))
    return pos, yaw, roll, pitch


class Evaluation:
    """Collects the metrics of an evaluation run: warm-up cycles, then a measurement window of whole cycles."""

    def __init__(self, path, params, warmup_s, measure_s):
        period = params["gait_period"]
        self.path = path
        self.params = params
        self.t_start = max(1, math.ceil(warmup_s / period)) * period
        self.t_end = self.t_start + max(2, math.ceil(measure_s / period)) * period
        self.wall_start = time.time()
        self.targets_deg = []
        self.trace = []
        self.next_trace = 0.0
        self.window_start = None
        self.tilt_sq = []
        self.tilt_max = 0.0
        self.tilt_max_all = 0.0
        self.z = []
        self.torque = []
        self.done = False

    def record(self, t, pos, yaw, roll, pitch, omega, targets):
        """Called once per walking step. Returns True when the evaluation is over."""
        self.targets_deg.append(np.degrees(targets))
        tilt_deg = math.degrees(max(abs(roll), abs(pitch)))
        self.tilt_max_all = max(self.tilt_max_all, tilt_deg)
        if t >= self.next_trace:
            self.trace.append([round(t, 2), round(pos[0], 4), round(pos[1], 4), round(math.degrees(yaw), 2),
                               round(math.degrees(roll), 2), round(math.degrees(pitch), 2), round(omega, 4)])
            self.next_trace += 0.5

        if tilt_deg >= FALL_TILT_DEG or pos[2] < FALL_HEIGHT:
            self.finish(fell=True, t=t)
            return True

        if t >= self.t_start:
            if self.window_start is None:
                self.window_start = (t, pos[0], pos[1], yaw)
            self.tilt_sq.append(math.degrees(roll)**2 + math.degrees(pitch)**2)
            self.tilt_max = max(self.tilt_max, tilt_deg)
            self.z.append(pos[2])
            self.torque.append([abs(m.getTorqueFeedback()) for m in motors])

        if t >= self.t_end:
            self.finish(fell=False, t=t, end=(t, pos[0], pos[1], yaw))
            return True
        return False

    def finish(self, fell, t, end=None):
        result = {
            "valid": True,
            "fell": fell,
            "t_gait": round(t, 2),
            "t_window": [self.t_start, self.t_end],
            "tilt_max_all_deg": self.tilt_max_all,
            "wall_s": round(time.time() - self.wall_start, 2),
            "params": self.params,
            "trace": self.trace,
        }
        if not fell:
            t0, x0, y0, yaw0 = self.window_start
            t1, x1, y1, yaw1 = end
            torque = np.array(self.torque)
            deg = np.array(self.targets_deg)
            result.update({
                "window_s": t1 - t0,
                "dx": x1 - x0,
                "dy": y1 - y0,
                "v_forward": (x1 - x0) / (t1 - t0),
                "v_lateral": (y1 - y0) / (t1 - t0),
                "yaw_change_deg": math.degrees(wrap(yaw1 - yaw0)),
                "tilt_rms_deg": math.sqrt(float(np.mean(self.tilt_sq))),
                "tilt_max_deg": self.tilt_max,
                "z_mean": float(np.mean(self.z)),
                "torque_rms": float(np.sqrt(np.mean(torque**2))),
                "torque_p99": float(np.percentile(torque, 99)),
                "joint_min_deg": deg.min(axis=0).tolist(),
                "joint_max_deg": deg.max(axis=0).tolist(),
                "max_joint_speed_deg_s": peak_joint_speed(deg, dt),
            })
        write_result(self.path, result)


def write_result(path, result):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(result, f)
    os.replace(tmp, path)  # the tuner never sees a half-written file


def main():
    params = load_params()
    gen = GaitGenerator(params, dt)
    yaw_enabled = params["yaw_kp"] > 0 or params["yaw_ki"] > 0
    yaw_ctrl = YawController(params["yaw_kp"], params["yaw_ki"], params["yaw_max"], math.radians(params["yaw_ref_deg"]))
    omega = 0.0

    result_path = os.environ.get("GAIT_RESULT")
    evaluation = None
    if result_path:
        evaluation = Evaluation(result_path, params, float(os.environ.get("GAIT_WARMUP_S", 6)),
                                float(os.environ.get("GAIT_MEASURE_S", 24)))

    def command():
        for motor, value in zip(motors, gen.targets):
            motor.setPosition(float(value))
        if yaw_enabled:  # otherwise the shoulders are left alone, as in the original gait
            for shoulder, value in zip(shoulders, gen.shoulder_targets):
                shoulder.setPosition(float(value))

    command()  # standing pose
    delay = params["init_delay"]

    while supervisor.step(timestep) != -1:
        if delay > 0:
            delay -= dt
            continue

        pos, yaw, roll, pitch = read_pose()

        previous_stage = gen.stage
        gen.step()
        if yaw_enabled and gen.push_just_finished(previous_stage):
            omega = yaw_ctrl.update(yaw)
            gen.set_twist(omega)
        command()

        if evaluation:
            if evaluation.record(gen.t, pos, yaw, roll, pitch, omega, gen.targets):
                supervisor.simulationQuit(0)
                return
        elif gen.stage == 0 and previous_stage != 0:  # once per cycle
            print(f"[gait] t={gen.t:6.1f} s  x={pos[0]:6.3f} m  y={pos[1]:6.3f} m  "
                  f"yaw={math.degrees(yaw):6.2f} deg  turn command={omega:+.3f} rad/cycle", flush=True)


if __name__ == "__main__":
    result_path = os.environ.get("GAIT_RESULT")
    try:
        main()
    except Exception as error:  # in evaluation mode a crash (e.g. a foot out of reach) must still produce a result file
        if not result_path:
            raise
        write_result(result_path, {"valid": False, "error": f"{type(error).__name__}: {error}"})
        supervisor.simulationQuit(1)
