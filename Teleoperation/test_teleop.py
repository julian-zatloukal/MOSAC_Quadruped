"""Tests of the Teleoperation chain without Webots: protocol, the web server, and the robot side (RemoteControl driving a
real TurnController), connected over loopback as they are in use: WebRTC from a page (here an aiortc client that connects the way
the web app does) to the server, UDP from there to the robot.
Usage: python test_teleop.py        (needs aiortc, see requirements.txt)
"""
import asyncio
import http.client
import json
import math
import os
import socket
import sys
import threading
import time

from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription

HERE = os.path.dirname(os.path.abspath(__file__))
YAW = os.path.join(HERE, "..", "Webots", "controllers", "Programmed_Yaw")
sys.path.insert(0, HERE)
sys.path.insert(0, YAW)

import protocol  # noqa: E402
import server  # noqa: E402
import webrtc  # noqa: E402
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


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


class Browser:
    """A page, without the picture: it connects to the server over WebRTC the way the web app does (an offer with a video
    transceiver and the data channel "teleop", posted to /api/offer), from a thread of its own."""

    def __init__(self, port):
        self.port = port
        self.state = None  # the last state the server pushed
        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._loop.run_forever, daemon=True).start()
        self._run(self._connect())

    def _run(self, coroutine):
        return asyncio.run_coroutine_threadsafe(coroutine, self._loop).result(10)

    def _signal(self, sdp):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        connection.request("POST", "/api/offer", json.dumps({"sdp": sdp, "type": "offer"}),
                           {"Content-Type": "application/json", "Origin": f"http://127.0.0.1:{self.port}"})
        response = connection.getresponse()
        body = response.read()
        connection.close()
        assert response.status == 200, (response.status, body)
        return json.loads(body)

    async def _connect(self):
        self.pc = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        self.channel = self.pc.createDataChannel(webrtc.CHANNEL_LABEL, ordered=True, maxPacketLifeTime=200)
        self.pc.addTransceiver("video", direction="recvonly")
        opened = asyncio.Event()
        self.channel.on("open", opened.set)
        self.channel.on("message", lambda message: setattr(self, "state", json.loads(message)))
        await self.pc.setLocalDescription(await self.pc.createOffer())
        answer = await self._loop.run_in_executor(None, self._signal, self.pc.localDescription.sdp)
        await self.pc.setRemoteDescription(RTCSessionDescription(**answer))
        await asyncio.wait_for(opened.wait(), 10)

    def send(self, payload):
        self._loop.call_soon_threadsafe(self.channel.send, json.dumps(payload) if not isinstance(payload, bytes) else payload)

    def wait_state(self, until, seconds=2.0):
        """The state the server pushed once until(state) is true; fails after `seconds`."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.state is not None and until(self.state):
                return self.state
            time.sleep(0.01)
        raise AssertionError(f"the state never matched; last: {self.state}")

    def close(self):
        self._run(self.pc.close())
        self._loop.call_soon_threadsafe(self._loop.stop)


class Chain:
    """The web server, a page connected to it and a RemoteControl, linked over loopback on free ports. (The camera port is one
    nothing listens on: a Webots window running the world has the real one.)"""

    def __init__(self):
        self.httpd, self.link = server.make_server(port=0, command_port=0, status_port=0, video_port=free_port())
        self.motion = new_motion()
        self.clock = FakeClock()
        self.log = []
        self.robot = RemoteControl(self.motion, command_port=0, server=("127.0.0.1", self.link.status_port),
                                   clock=self.clock, log=self.log.append)
        self.link.robot = ("127.0.0.1", self.robot.command_port)
        self.port = self.httpd.server_port
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.page = Browser(self.port)
        self.yaw = 0.0
        self.position = (0.0, 0.0)

    def run_gait(self, steps=300):
        """Let the controller's ramps run (the gait itself is not stepped, only the controllers)."""
        for _ in range(steps):
            self.motion.update(0.0)

    def close(self):
        self.page.close()
        self.httpd.shutdown()
        self.httpd.server_close()
        self.httpd.pages.close()
        self.httpd.video.close()
        self.link.close()
        self.robot.close()

    def request(self, method, path, body=None, headers=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        connection.request(method, path, body=body, headers=headers or {})
        response = connection.getresponse()
        data = response.read()
        connection.close()
        return response.status, data

    def post(self, payload):
        """A command from the page."""
        self.page.send(payload)

    def offer(self, payload, headers=None):
        """A connection offer posted by hand, for the requests the server has to refuse."""
        headers = dict({"Content-Type": "application/json"}, **(headers or {}))
        return self.request("POST", "/api/offer", json.dumps(payload) if not isinstance(payload, bytes) else payload, headers)

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
        if os.path.isfile(server.PAGE):
            assert status == 200 and b"<title>MOSAC Teleoperation</title>" in body
        else:
            assert status == 503 and b"pnpm build" in body, "an unbuilt page must say how to build it"
        assert chain.request("GET", "/nope")[0] == 404
        assert chain.request("GET", "/../server.py")[0] == 404
        assert chain.request("GET", "/api/offer")[0] == 404

        state = chain.page.wait_state(lambda state: True)  # pushed over the data channel
        assert state["connected"] is False and state["robot"] is None

        chain.yaw = math.radians(33.0)
        chain.position = (0.1234, -0.05)
        chain.update()
        state = chain.page.wait_state(lambda state: state["connected"])
        robot = state["robot"]
        assert robot["yaw_deg"] == 33.0 and (robot["x_m"], robot["y_m"]) == (0.123, -0.05), state
        assert robot["mode"] == "stop" and robot["idle"] is True and robot["forward"] == 0.0
        assert robot["max_rate_deg_s"] > 0
    finally:
        chain.close()


def test_turn_commands_reach_the_robot():
    chain = Chain()
    try:
        chain.post({"cmd": "drive", "forward": 0.0, "turn": 0.5})
        assert chain.update(lambda: chain.motion.mode == "turn"), "turn command never arrived"
        chain.run_gait()  # let the ramp speed it up
        assert 0 < chain.motion.active.omega <= 0.5 * chain.motion.turner.omega_max + 1e-9
        assert any("turn left 50%" in line for line in chain.log), chain.log

        chain.post({"cmd": "drive", "forward": 0.0, "turn": -0.25})  # a held button sends this again and again
        assert chain.update(lambda: chain.motion.turner._pending is not None or chain.motion.active.omega < 0)
        assert chain.motion.active.omega == 0.0, "a reversal must stop the rotation at once"

        chain.post({"cmd": "stop"})
        assert chain.update(lambda: chain.motion.mode == "stop" and chain.motion.turner._pending is None)
        assert chain.motion.active.omega == 0.0
        assert chain.log[-1] == "[teleop] stop", chain.log
    finally:
        chain.close()


def test_walk_commands_reach_the_robot():
    chain = Chain()
    try:
        chain.post({"cmd": "drive", "forward": 1.0, "turn": 0.0})
        assert chain.update(lambda: chain.motion.mode == "walk"), "walk command never arrived"
        chain.run_gait(3000)
        assert chain.motion.forward == 1.0
        assert any("walk 100%" in line for line in chain.log), chain.log
        chain.post({"cmd": "drive", "forward": 0.5, "turn": -1.0})
        assert chain.update(lambda: any("walk 50%, turn right 100%" in line for line in chain.log))
        assert chain.motion.mode == "walk"

        # walking backwards while walking forward: the walk stops and the other waits for the legs to centre
        chain.post({"cmd": "drive", "forward": -0.5, "turn": 0.0})
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
        chain.post({"cmd": "drive", "forward": 0.0, "turn": 1.0})
        assert chain.update(lambda: chain.motion.mode == "stop" and chain.motion.forward == 0.0)
        assert chain.motion._pending == (0.0, 1.0)
        chain.post({"cmd": "stop"})
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
        pages = lambda: len(chain.httpd.pages._pages)
        assert pages() == 1  # the page of this chain
        offer = {"sdp": "v=0\r\n", "type": "offer"}
        assert chain.offer(offer, {"Content-Type": "text/plain"})[0] == 415
        assert chain.offer(offer, {"Origin": "http://evil.example"})[0] == 403
        assert chain.offer(offer, {"Host": "evil.example"})[0] == 403
        assert chain.request("GET", "/", headers={"Host": "evil.example"})[0] == 403
        assert chain.offer(b"not json")[0] == 400
        assert chain.offer({"cmd": "drive"})[0] == 400
        assert chain.offer({"sdp": 5, "type": "offer"})[0] == 400
        assert chain.offer({"sdp": "v=0\r\n", "type": "answer"})[0] == 400
        assert chain.offer(b"x" * (server.MAX_BODY + 1))[0] == 400
        assert chain.offer(offer)[0] == 400, "an offer with no data channel must be refused"
        assert chain.offer({"sdp": "v=0\r\nm=application 9 UDP/DTLS/SCTP webrtc-datachannel\r\n", "type": "offer"})[0] == 400
        assert chain.request("POST", "/api/command", b"{}", {"Content-Type": "application/json"})[0] == 404
        assert chain.request("GET", "/api/state")[0] == 404
        time.sleep(0.1)
        assert pages() == 1, "a refused offer still made a connection"

        chain.update()
        time.sleep(0.05)
        chain.update()
        assert chain.motion.mode == "stop" and chain.log == [], "a refused request still reached the robot"
        chain.post(b"not a command")  # what the data channel carries has to be a command too
        chain.post({"cmd": "dance"})
        chain.post({"cmd": "drive", "forward": "1"})
        chain.update()
        time.sleep(0.1)
        chain.update()
        assert chain.motion.mode == "stop" and chain.log == [], "an invalid command still reached the robot"

        # a valid one, from the page's own origin, does
        chain.post({"cmd": "drive", "forward": 1.0, "turn": 0.0})
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
