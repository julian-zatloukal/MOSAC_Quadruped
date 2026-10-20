"""Messages between the Teleoperation web server and the robot (the Webots controller Programmed_Yaw --teleop).

UDP, one JSON object per datagram. Imported by both sides, so the ports and the timeout live in one place.

  server -> robot, to COMMAND_PORT:
      {"cmd": "drive", "forward": 1.0, "turn": -0.5}
                  walk forward at this fraction of the full stride (0..1) while turning at this fraction of the full turn
                  (-1..1, positive = to the left). With forward 0 it turns on the spot, with both 0 it stops.
      {"cmd": "stop"}                halt
  A drive only lasts COMMAND_TIMEOUT seconds: the sender has to repeat it while the button is held, and the robot stops by itself
  when the repeats stop (page closed, network lost, server killed).

  robot -> server, to STATUS_PORT, every STATUS_PERIOD seconds:
      {"yaw_deg", "x_m", "y_m", "mode", "idle", "forward", "rate_deg_s", "max_rate_deg_s"}   see remote.RemoteControl.status()
"""
import json
import math

COMMAND_PORT = 9871    # the robot listens here
STATUS_PORT = 9872     # the web server listens here
COMMAND_TIMEOUT = 0.5  # s without a drive command and the robot stops
STATUS_PERIOD = 0.1    # s between status messages


def encode_command(cmd, forward=0.0, turn=0.0):
    if cmd == "drive":
        return json.dumps({"cmd": "drive", "forward": forward, "turn": turn}).encode()
    if cmd == "stop":
        return json.dumps({"cmd": "stop"}).encode()
    raise ValueError(f"unknown command {cmd!r}")


def _fraction(value, low):
    """value clamped to [low, 1], or None if it is not a finite number."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None
    return max(low, min(1.0, float(value)))


def decode_command(data):
    """("drive", forward, turn) with forward clamped to [0, 1] and turn to [-1, 1], ("stop", 0.0, 0.0), or None for anything
    that is not a valid command. A missing forward or turn counts as 0."""
    try:
        message = json.loads(data)
    except ValueError:
        return None
    if not isinstance(message, dict):
        return None
    if message.get("cmd") == "stop":
        return ("stop", 0.0, 0.0)
    if message.get("cmd") == "drive":
        forward, turn = _fraction(message.get("forward", 0.0), 0.0), _fraction(message.get("turn", 0.0), -1.0)
        return None if forward is None or turn is None else ("drive", forward, turn)
    return None


def encode_status(**fields):
    return json.dumps(fields).encode()


def decode_status(data):
    """The status dict, or None if the datagram is not one."""
    try:
        status = json.loads(data)
    except ValueError:
        return None
    return status if isinstance(status, dict) else None
