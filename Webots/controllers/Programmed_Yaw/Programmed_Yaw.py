"""Programmed_Yaw controller.

The crawl gait of Programmed_Gait_V2 turning on the spot, and a scripted session that shows it: hold a turn, stop, go to an
angle, stop, go to another angle (SCRIPT below, progress is printed to the console).

The commands come from Session, which calls the three commands of TurnController:
    turner.turn(rate)      hold a turn, rate in [-1, 1] of the full speed (0 = stop)
    turner.go_to(heading)  turn to a heading in rad (world frame, 0 = +x, positive = to the left) and stop there
    turner.stop()          halt
With --teleop (controllerArgs in MOSAC_REAL_V4_teleop.wbt) the commands come from the Teleoperation web page instead
(remote.py, UDP port 9871), and the robot can also walk forward and backwards (motion_core.py); TELEOP_BIND and TELEOP_SERVER_HOST in the
environment move the robot's side of the link off 127.0.0.1. The webcam on the robot ("camera" in that world) is streamed to the
page while it shows the video (camera_stream.py, H.264 through ffmpeg, TCP port 9873).

How fast it turns is set in turn_params.json (see README.md). TURN_PARAMS=<file> uses another file.
The heading is read from the simulator (a stand-in for the gyro of the IMU on the real robot).
"""
import math
import os
import sys

from controller import Supervisor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "Programmed_Gait_V2"))

from gait_core import GaitGenerator, JOINT_NAMES, SHOULDER_NAMES  # noqa: E402
from turn_core import TurnController, load_turn_params  # noqa: E402

STATUS_PORT = 9872  # Teleoperation/protocol.py; only the remote mode needs that module, so the number is repeated here

RESET_POS = [0.0, 0.0, 0.141]

# (command, value, seconds)
#   turn   hold a turn at value (fraction of the full speed) for this many seconds
#   stop   stop, and wait until the legs are centred and at least this many seconds have passed
#   go_to  turn to the world heading value (deg); the next line starts when it arrives
SCRIPT = [
    ("turn", 1.0, 5.0),
    ("stop", None, 3.0),
    ("go_to", 90.0, None),
    ("stop", None, 2.0),
    ("go_to", -45.0, None),
    ("stop", None, 2.0),
]

supervisor = Supervisor()
robot_node = supervisor.getFromDef('MOSAC')
timestep = int(supervisor.getBasicTimeStep())
dt = timestep * 1e-3

robot_node.getField("translation").setSFVec3f(RESET_POS)
robot_node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, 0.0])

motors = [supervisor.getDevice(name) for name in JOINT_NAMES]
shoulders = [supervisor.getDevice(name) for name in SHOULDER_NAMES]


def read_pose():
    """(x, y) in m and yaw in rad."""
    position = robot_node.getPosition()
    R = robot_node.getOrientation()
    return (position[0], position[1]), math.atan2(R[3], R[0])


class Session:
    """Plays SCRIPT: update() once per control step."""

    def __init__(self, turner):
        self.turner = turner
        self.line = -1
        self.clock = 0.0   # s since the current line started
        self.total = 0.0
        self.yaw = 0.0
        self.advance()

    def say(self, text):
        print(f"[yaw] t={self.total:6.1f} s  yaw={math.degrees(self.yaw):7.1f} deg  {text}", flush=True)

    def advance(self):
        self.line += 1
        self.clock = 0.0
        if self.line >= len(SCRIPT):
            self.say("script finished, the robot stays where it is")
            return
        command, value, _ = SCRIPT[self.line]
        if command == "turn":
            self.turner.turn(value)
            self.say(f"turn {'left' if value > 0 else 'right'} at {abs(value):.0%} of the full speed")
        elif command == "stop":
            self.turner.stop()
            self.say("stop (rotation halts, feet re-centre)")
        else:
            self.turner.go_to(math.radians(value))
            self.say(f"go to {value:+.0f} deg")

    def update(self, yaw, position=None):
        self.yaw = yaw
        self.total += dt
        self.clock += dt
        if self.line >= len(SCRIPT):
            return
        command, _, seconds = SCRIPT[self.line]
        if command == "turn" and self.clock >= seconds:
            self.advance()
        elif command == "stop" and self.turner.idle and self.clock >= seconds:
            self.say("feet re-centred, standing still")
            self.advance()
        elif command == "go_to" and self.turner.mode == "stop":
            self.say("arrived")
            self.advance()


def main():
    params, settings = load_turn_params()
    gen = GaitGenerator(params, dt)
    turner = TurnController(gen, settings["omega_max"], settings["ramp_cycles"], settings["kp"], settings["omega_min"],
                            math.radians(settings["tol_deg"]))
    print(f"[yaw] gait period {params['gait_period']} s, foot lift {params['parabola_height']} mm, "
          f"full speed {turner.max_rate_deg_s:.1f} deg/s", flush=True)

    def command():
        for motor, value in zip(motors, gen.targets):
            motor.setPosition(float(value))
        for shoulder, value in zip(shoulders, gen.shoulder_targets):
            shoulder.setPosition(float(value))

    command()  # standing pose
    delay = params["init_delay"]
    controller = turner  # what the gait follows: the turner, or the MotionController that also walks
    source = None  # where the commands come from: the script, or the web page
    video = None   # the camera stream of the web page
    if "--teleop" in sys.argv[1:]:
        from motion_core import add_walking
        from remote import RemoteControl
        controller = add_walking(gen, turner)
        bind_host = os.environ.get("TELEOP_BIND", "127.0.0.1")
        source = RemoteControl(controller, bind_host=bind_host,
                               server=(os.environ.get("TELEOP_SERVER_HOST", "127.0.0.1"), STATUS_PORT))
        print(f"[teleop] listening for commands on UDP {source.command_port}", flush=True)
        camera = supervisor.getDevice("camera")
        if camera is not None:
            from camera_stream import start_camera
            video = start_camera(camera, timestep, bind_host, log=lambda text: print(text, flush=True))

    while supervisor.step(timestep) != -1:
        if video is not None:
            video.update(round(supervisor.getTime() * 1000))
        position, yaw = read_pose()
        if source is None and delay <= 0:  # the script starts when the robot has finished standing up
            source = Session(turner)
        if source is not None:
            source.update(yaw, position)
        if delay > 0:
            delay -= dt
            continue

        previous_stage = gen.stage
        if not controller.idle:  # stopped and centred: hold the pose
            gen.step()
        controller.update(yaw, gen.push_just_finished(previous_stage))
        command()


if __name__ == "__main__":
    main()
