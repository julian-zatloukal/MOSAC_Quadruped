"""Tests of the Teleoperation chain without Webots: protocol, the web server, and the robot side (RemoteControl driving a
real TurnController), connected over loopback UDP and HTTP exactly as they are in use.
Usage: python test_teleop.py
"""
import http.client
import json
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
YAW = os.path.join(HERE, "..", "Webots", "controllers", "Programmed_Yaw")
sys.path.insert(0, HERE)
sys.path.insert(0, YAW)

import protocol  # noqa: E402
import server  # noqa: E402
from remote import RemoteControl  # noqa: E402
from motion_core import add_walking  # noqa: E402
from turn_core import TurnController, load_turn_params  # noqa: E402
from gait_core import GaitGenerator  # noqa: E402  (turn_core puts Programmed_Gait_V2 on the path)


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def new_motion():
    params, settings = load_turn_params()
    gen = GaitGenerator(params, 0.01)
    turner = TurnController(gen, settings["omega_max"], settings["ramp_cycles"], settings["kp"], settings["omega_min"],
                            math.radians(settings["tol_deg"]))
    return add_walking(gen, turner)


class Chain:
    """The web server and a RemoteControl, linked over loopback on free ports."""

    def __init__(self):
        self.httpd, self.link = server.make_server(port=0, command_port=0, status_port=0)
        self.motion = new_motion()
        self.clock = FakeClock()
        self.log = []
        self.robot = RemoteControl(self.motion, command_port=0, server=("127.0.0.1", self.link.status_port),
                                   clock=self.clock, log=self.log.append)
        self.link.robot = ("127.0.0.1", self.robot.command_port)
        self.port = self.httpd.server_port
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.yaw = 0.0
        self.position = (0.0, 0.0)

    def run_gait(self, steps=300):
        """Let the controller's ramps run (the gait itself is not stepped, only the controllers)."""
        for _ in range(steps):
            self.motion.update(0.0)

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.link.close()
        self.robot.close()

    def request(self, method, path, body=None, headers=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        connection.request(method, path, body=body, headers=headers or {})
        response = connection.getresponse()
        data = response.read()
        connection.close()
        return response.status, data

    def post(self, payload, headers=None):
        headers = dict({"Content-Type": "application/json"}, **(headers or {}))
        return self.request("POST", "/api/command", json.dumps(payload) if not isinstance(payload, bytes) else payload, headers)

    def update(self, until=None, seconds=1.0):
        """Run the robot side until until() is true (or one update when until is None). Returns whether it became true."""
        deadline = time.monotonic() + seconds
        while True:
            self.robot.update(self.yaw, self.position)
            if until is None or until():
                return True
            if time.monotonic() > deadline:
                return False
            time.sleep(0.005)


def test_protocol():
    assert protocol.decode_command(protocol.encode_command("drive", 0.5, -0.25)) == ("drive", 0.5, -0.25)
    assert protocol.decode_command(protocol.encode_command("stop")) == ("stop", 0.0, 0.0)
    assert protocol.decode_command(b'{"cmd": "drive", "forward": 7, "turn": -7}') == ("drive", 1.0, -1.0), "must be clamped"
    assert protocol.decode_command(b'{"cmd": "drive", "forward": -3}') == ("drive", -1.0, 0.0), "must be clamped"
    assert protocol.decode_command(protocol.encode_command("drive", -0.5, 0.0)) == ("drive", -0.5, 0.0)
    assert protocol.decode_command(b'{"cmd": "drive", "turn": 0.5}') == ("drive", 0.0, 0.5), "forward defaults to 0"
    for junk in (b"", b"drive", b"[1]", b'{"cmd": "jump"}', b'{"cmd": "turn", "rate": 1}', b'{"cmd": "drive", "forward": "1"}',
                 b'{"cmd": "drive", "turn": true}', b'{"cmd": "drive", "forward": NaN}', b'{"cmd": "drive", "turn": Infinity}',
                 b"\xff\xfe"):
        assert protocol.decode_command(junk) is None, junk
    assert protocol.decode_status(b'{"yaw_deg": 3}') == {"yaw_deg": 3}
    assert protocol.decode_status(b"[]") is None and protocol.decode_status(b"x") is None


def test_page_and_state():
    chain = Chain()
    try:
        status, body = chain.request("GET", "/")
        assert status == 200 and b"<title>MOSAC Teleoperation</title>" in body
        status, body = chain.request("GET", "/api/state")
        state = json.loads(body)
        assert status == 200 and state["connected"] is False and state["robot"] is None
        assert chain.request("GET", "/nope")[0] == 404
        assert chain.request("GET", "/../server.py")[0] == 404

        chain.yaw = math.radians(33.0)
        chain.position = (0.1234, -0.05)
        chain.update()
        for _ in range(100):
            state = json.loads(chain.request("GET", "/api/state")[1])
            if state["connected"]:
                break
            time.sleep(0.01)
        robot = state["robot"]
        assert state["connected"] and robot["yaw_deg"] == 33.0 and (robot["x_m"], robot["y_m"]) == (0.123, -0.05), state
        assert robot["mode"] == "stop" and robot["idle"] is True and robot["forward"] == 0.0
        assert robot["max_rate_deg_s"] > 0
    finally:
        chain.close()


def test_turn_commands_reach_the_robot():
    chain = Chain()
    try:
        assert chain.post({"cmd": "drive", "forward": 0.0, "turn": 0.5})[0] == 200
        assert chain.update(lambda: chain.motion.mode == "turn"), "turn command never arrived"
        chain.run_gait()  # let the ramp speed it up
        assert 0 < chain.motion.active.omega <= 0.5 * chain.motion.turner.omega_max + 1e-9
        assert any("turn left 50%" in line for line in chain.log), chain.log

        assert chain.post({"cmd": "drive", "forward": 0.0, "turn": -0.25})[0] == 200  # a held button sends this again and again
        assert chain.update(lambda: chain.motion.turner._pending is not None or chain.motion.active.omega < 0)
        assert chain.motion.active.omega == 0.0, "a reversal must stop the rotation at once"

        assert chain.post({"cmd": "stop"})[0] == 200
        assert chain.update(lambda: chain.motion.mode == "stop" and chain.motion.turner._pending is None)
        assert chain.motion.active.omega == 0.0
        assert chain.log[-1] == "[teleop] stop", chain.log
    finally:
        chain.close()


def test_walk_commands_reach_the_robot():
    chain = Chain()
    try:
        assert chain.post({"cmd": "drive", "forward": 1.0, "turn": 0.0})[0] == 200
        assert chain.update(lambda: chain.motion.mode == "walk"), "walk command never arrived"
        chain.run_gait(3000)
        assert chain.motion.forward == 1.0
        assert any("walk 100%" in line for line in chain.log), chain.log
        assert chain.post({"cmd": "drive", "forward": 0.5, "turn": -1.0})[0] == 200
        assert chain.update(lambda: any("walk 50%, turn right 100%" in line for line in chain.log))
        assert chain.motion.mode == "walk"

        # walking backwards while walking forward: the walk stops and the other waits for the legs to centre
        assert chain.post({"cmd": "drive", "forward": -0.5, "turn": 0.0})[0] == 200
        assert chain.update(lambda: chain.motion._pending == (-0.5, 0.0))
        assert chain.motion.mode == "stop" and chain.motion.forward == 0.0
        chain.motion.walker.idle = True  # the legs re-centred (the gait is not stepped here)
        chain.run_gait(3000)
        assert chain.motion.mode == "back" and chain.motion.forward == -0.5
        assert any("walk back 50%" in line for line in chain.log), chain.log
        state = chain.robot.status(0.0)
        assert state["mode"] == "back" and state["forward"] == -0.5, state
        turn_params = chain.motion._params["turn"]  # the turn on the spot, whatever gait is running
        assert state["max_rate_deg_s"] == round(math.degrees(chain.motion.turner.omega_max) / turn_params["gait_period"], 2)

        # turning on the spot while walking: the walk stops and the turn waits for the legs to centre
        assert chain.post({"cmd": "drive", "forward": 0.0, "turn": 1.0})[0] == 200
        assert chain.update(lambda: chain.motion.mode == "stop" and chain.motion.forward == 0.0)
        assert chain.motion._pending == (0.0, 1.0)
        assert chain.post({"cmd": "stop"})[0] == 200
        assert chain.update(lambda: chain.motion._pending is None)
    finally:
        chain.close()


def test_the_robot_stops_when_the_commands_stop():
    chain = Chain()
    try:
        chain.post({"cmd": "drive", "forward": 1.0, "turn": 0.0})
        assert chain.update(lambda: chain.motion.mode == "walk")
        chain.run_gait(3000)
        assert chain.motion.forward == 1.0

        chain.clock.now += protocol.COMMAND_TIMEOUT * 0.8  # still inside the timeout
        chain.update()
        assert chain.motion.mode == "walk"
        chain.post({"cmd": "drive", "forward": 1.0, "turn": 0.0})  # the repeat of a held button restarts the timeout
        assert chain.update(lambda: chain.robot._last_drive == chain.clock.now), "the repeat never arrived"
        chain.clock.now += protocol.COMMAND_TIMEOUT * 0.8
        chain.update()
        assert chain.motion.mode == "walk", "the repeat did not extend the drive"

        chain.clock.now += protocol.COMMAND_TIMEOUT * 0.5  # now the repeats have been missing for too long
        chain.update()
        assert chain.motion.mode == "stop" and chain.motion.forward == 0.0
        assert "no command for" in chain.log[-1], chain.log

        # A command that was waiting for the legs to centre is cancelled by the timeout too
        chain.post({"cmd": "drive", "forward": 1.0, "turn": 0.0})
        chain.update(lambda: chain.motion.mode == "walk")
        chain.run_gait(3000)
        chain.post({"cmd": "drive", "forward": 0.0, "turn": 1.0})
        chain.update(lambda: chain.motion._pending is not None)
        chain.clock.now += protocol.COMMAND_TIMEOUT * 1.5
        chain.update()
        assert chain.motion._pending is None and chain.motion.mode == "stop"
    finally:
        chain.close()


def test_the_server_refuses_what_it_should():
    chain = Chain()
    try:
        body = {"cmd": "drive", "forward": 1.0, "turn": 0.0}
        assert chain.post(body, {"Content-Type": "text/plain"})[0] == 415
        assert chain.post(body, {"Origin": "http://evil.example"})[0] == 403
        assert chain.post(body, {"Host": "evil.example"})[0] == 403
        assert chain.request("GET", "/api/state", headers={"Host": "evil.example"})[0] == 403
        assert chain.post(b"not json")[0] == 400
        assert chain.post({"cmd": "dance"})[0] == 400
        assert chain.post(b"x" * (server.MAX_BODY + 1))[0] == 400
        assert chain.request("POST", "/api/other", b"{}", {"Content-Type": "application/json"})[0] == 404
        assert chain.request("GET", "/api/command")[0] == 404
        chain.update()
        time.sleep(0.05)
        chain.update()
        assert chain.motion.mode == "stop" and chain.log == [], "a refused request still reached the robot"
        # the page's own origin is accepted
        assert chain.post(body, {"Origin": f"http://127.0.0.1:{chain.port}"})[0] == 200
        assert chain.update(lambda: chain.motion.mode == "walk")
    finally:
        chain.close()


def main():
    tests = [test_protocol, test_page_and_state, test_turn_commands_reach_the_robot, test_walk_commands_reach_the_robot,
             test_the_robot_stops_when_the_commands_stop, test_the_server_refuses_what_it_should]
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
