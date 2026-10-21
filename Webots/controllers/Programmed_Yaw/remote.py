"""Commands from the Teleoperation web server (../../../Teleoperation) for a MotionController. Pure python (no Webots).

RemoteControl listens on a UDP port for the messages of Teleoperation/protocol.py and passes them to the MotionController, and
reports the heading and position back to the server. It is the other choice besides Session in Programmed_Yaw.py (run with
--teleop).

A drive command only lasts protocol.COMMAND_TIMEOUT seconds, so the sender has to keep repeating it while the button is held.
When the repeats stop (the page closed, the network went down, the server was killed) the robot stops by itself. The time is
wall clock time: Webots can run much faster or slower than real time, but the commands arrive in real time.
"""
import math
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "Teleoperation"))

import protocol  # noqa: E402


class RemoteControl:
    def __init__(self, motion, command_port=protocol.COMMAND_PORT, server=("127.0.0.1", protocol.STATUS_PORT),
                 bind_host="127.0.0.1", clock=time.monotonic, log=print):
        """motion: the MotionController to drive. command_port, bind_host: where to listen for commands. server: where the
        status goes. clock: seconds, for the command timeout and the status period. log: called with a line of text."""
        self.motion = motion
        self.server = server
        self._clock = clock
        self._log = log
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._sock.bind((bind_host, command_port))
        except OSError as error:
            self._sock.close()
            raise OSError(f"cannot listen for teleoperation commands on UDP {bind_host}:{command_port} ({error}); "
                          f"is another Webots window running this world?") from error
        self._sock.setblocking(False)
        self.command_port = self._sock.getsockname()[1]
        self._last_drive = None   # when the drive command in force arrived; None while nothing is held
        self._next_status = 0.0
        self._shown = None

    def close(self):
        self._sock.close()

    def status(self, yaw, position=None):
        """What the page shows: heading (deg), position (m) if known, the MotionController mode ("stop", "turn", "walk" or
        "back"), whether the legs are centred and at rest, the stride being commanded (-1..1, negative = backwards), and the yaw
        rate being commanded and the full speed turn rate on the spot (deg/s)."""
        m = self.motion
        status = {"yaw_deg": round(math.degrees(yaw), 2), "mode": m.mode, "idle": m.idle, "forward": round(m.forward, 3),
                  "rate_deg_s": round(m.rate_deg_s, 2), "max_rate_deg_s": round(m.max_rate_deg_s, 2)}
        if position is not None:
            status["x_m"], status["y_m"] = round(position[0], 3), round(position[1], 3)
        return status

    def update(self, yaw, position=None):
        """Once per control step: apply the commands that arrived, stop if they ran out, and report. yaw in rad, position
        (x, y) in m."""
        now = self._clock()
        while True:
            try:
                data, _ = self._sock.recvfrom(4096)
            except OSError:  # nothing more to read (or the status port was closed on the other side)
                break
            command = protocol.decode_command(data)
            if command is None:
                continue
            kind, forward, turn = command
            if kind == "drive" and (forward != 0.0 or turn != 0.0):
                self._last_drive = now
                self.motion.drive(forward, turn)
                self._say(("drive", round(forward, 2), round(turn, 2)), self._describe(forward, turn))
            else:
                self._last_drive = None
                self.motion.stop()
                self._say(("stop",), "stop")

        if self._last_drive is not None and now - self._last_drive > protocol.COMMAND_TIMEOUT:
            self._last_drive = None
            self.motion.stop()
            self._say(("stop",), f"stop: no command for {protocol.COMMAND_TIMEOUT} s")

        if now >= self._next_status:
            self._next_status = now + protocol.STATUS_PERIOD
            try:
                self._sock.sendto(protocol.encode_status(**self.status(yaw, position)), self.server)
            except OSError:
                pass  # nobody is listening yet

    @staticmethod
    def _describe(forward, turn):
        parts = []
        if forward:
            parts.append(f"walk {'back ' if forward < 0 else ''}{abs(forward):.0%}")
        if turn:
            parts.append(f"turn {'left' if turn > 0 else 'right'} {abs(turn):.0%}")
        return ", ".join(parts)

    def _say(self, key, text):
        if key != self._shown:  # a held button repeats the same command ten times a second
            self._shown = key
            self._log(f"[teleop] {text}")
