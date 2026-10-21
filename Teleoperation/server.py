"""Teleoperation website: a page with the robot's camera and drive controls, hosted from here, that sends the commands to the
Webots robot.

    python server.py                    serves http://127.0.0.1:8000 and opens it in the browser
    python server.py --port 8001 --no-browser

Needs only the standard library. The robot side is the Webots world MOSAC_REAL_V4_teleop.wbt (controller Programmed_Yaw with
--teleop). Commands go to it over UDP (see protocol.py); it reports its heading back, which the page shows. While a page shows
the video, the server also connects to the robot's camera (TCP) and passes the H.264 frames to the page over a WebSocket.

The page is meant for this computer. With --host 0.0.0.0 anything on the network can drive the robot: there is no login.
"""
import argparse
import base64
import binascii
import hashlib
import json
import os
import select
import socket
import struct
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import protocol  # noqa: E402

FILES = {"/": ("index.html", "text/html; charset=utf-8"), "/index.html": ("index.html", "text/html; charset=utf-8"),
         "/video.js": ("video.js", "text/javascript; charset=utf-8")}
MAX_BODY = 1024      # bytes of a command request
STALE_AFTER = 1.0    # s without a status message before the robot counts as disconnected
VIDEO_LINGER = 3.0   # s the camera stays connected after the last page leaves (a reload does not restart the encoder)
WEBSOCKET_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


class RobotLink:
    """The UDP side: sends commands to the robot and keeps the last status it reported."""

    def __init__(self, robot_host, command_port, status_port, status_bind):
        self.robot = (robot_host, command_port)
        self._out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._in.bind((status_bind, status_port))
        self._in.settimeout(0.2)
        self.status_port = self._in.getsockname()[1]
        self._lock = threading.Lock()
        self._status = None
        self._time = 0.0
        self._closed = False
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def _listen(self):
        while not self._closed:
            try:
                data, _ = self._in.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            status = protocol.decode_status(data)
            if status is not None:
                with self._lock:
                    self._status, self._time = status, time.monotonic()

    def send(self, data):
        try:
            self._out.sendto(data, self.robot)
            return True
        except OSError:
            return False

    def state(self):
        with self._lock:
            age = time.monotonic() - self._time if self._status is not None else None
            return {"connected": age is not None and age < STALE_AFTER, "age": age, "robot": self._status}

    def close(self):
        self._closed = True
        self._in.close()
        self._out.close()
        self._thread.join(timeout=1)


class VideoRelay:
    """The camera: connected to the robot only while some page shows the video, it hands each message to every page's queue."""

    def __init__(self, robot_host, port, linger=VIDEO_LINGER):
        self.robot = (robot_host, port)
        self.linger = linger
        self._lock = threading.Lock()
        self._viewers = []
        self._config = None      # the robot's last config message: a page that joins needs it first
        self._last_frame = None  # time.monotonic() of the last frame
        self._wake = threading.Event()
        self._closed = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def subscribe(self):
        """A protocol.VideoQueue that gets the video until unsubscribe()."""
        queue = protocol.VideoQueue()
        with self._lock:
            self._viewers.append(queue)
            if self._config is not None:
                queue.put(self._config)
        self._wake.set()
        return queue

    def unsubscribe(self, queue):
        queue.close()
        with self._lock:
            if queue in self._viewers:
                self._viewers.remove(queue)

    def state(self):
        with self._lock:
            live = self._last_frame is not None and time.monotonic() - self._last_frame < STALE_AFTER
            return {"live": live, "viewers": len(self._viewers)}

    def close(self):
        self._closed = True
        self._wake.set()
        with self._lock:
            viewers, self._viewers = self._viewers, []
        for queue in viewers:
            queue.close()
        self._thread.join(timeout=2)

    def _watched(self):
        with self._lock:
            return bool(self._viewers)

    def _run(self):
        while not self._closed:
            if not self._watched():
                self._wake.wait()
                self._wake.clear()
                continue
            try:
                sock = socket.create_connection(self.robot, timeout=1.0)
            except OSError:  # no robot, or a robot without a camera: try again in a second
                self._wake.wait(1.0)
                self._wake.clear()
                continue
            try:
                self._relay(sock)
            finally:
                sock.close()
                with self._lock:
                    self._config = None

    def _relay(self, sock):
        """Until the robot goes away, or nobody has watched for `linger` seconds."""
        sock.settimeout(0.25)
        reader = protocol.VideoReader()
        alone_since = None
        while not self._closed:
            try:
                data = sock.recv(1 << 16)
            except socket.timeout:
                data = None
            except OSError:
                return
            if data == b"":
                return
            if data:
                try:
                    messages = reader.feed(data)
                except ValueError:
                    return
                with self._lock:
                    for message in messages:
                        if message[4] == protocol.VIDEO_CONFIG:
                            self._config = message
                        else:
                            self._last_frame = time.monotonic()
                    viewers = list(self._viewers)
                for message in messages:
                    for queue in viewers:
                        queue.put(message)
            if self._watched():
                alone_since = None
            elif alone_since is None:
                alone_since = time.monotonic()
            elif time.monotonic() - alone_since > self.linger:
                return  # the robot switches its camera off when this connection closes


def websocket_frame(payload, opcode=0x2):
    """A whole (FIN) server-to-page WebSocket frame; binary by default. Frames from a server are not masked."""
    n = len(payload)
    if n < 126:
        header = struct.pack(">BB", 0x80 | opcode, n)
    elif n < 1 << 16:
        header = struct.pack(">BBH", 0x80 | opcode, 126, n)
    else:
        header = struct.pack(">BBQ", 0x80 | opcode, 127, n)
    return header + payload


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"  # keep-alive: the page posts a command every 100 ms while a button is held
    server_version = "MOSACTeleop"

    def log_message(self, format, *args):
        pass  # 10 requests per second would bury everything else

    def _send(self, status, body, content_type="application/json"):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if self.close_connection:
            self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, status, payload):
        self._send(status, json.dumps(payload).encode())

    def _host_ok(self):
        """Refuses requests addressed to a name that is not ours, which is how a DNS rebinding attack reaches a local server."""
        allowed = self.server.allowed_hosts
        if allowed is None:
            return True
        host = self.headers.get("Host", "")
        name = host if host.endswith("]") or ":" not in host else host.rsplit(":", 1)[0]
        return name in allowed

    def do_GET(self):
        if not self._host_ok():
            self.close_connection = True
            return self._json(403, {"error": "unexpected Host header"})
        path = urlparse(self.path).path
        if path in FILES:
            name, content_type = FILES[path]
            with open(os.path.join(HERE, name), "rb") as f:
                self._send(200, f.read(), content_type)
        elif path == "/api/state":
            self._json(200, dict(self.server.link.state(), video=self.server.video.state()))
        elif path == "/api/video":
            self._video()
        else:
            self._json(404, {"error": "not found"})

    def _video(self):
        """The camera as a WebSocket: one binary message per protocol video message, without its length."""
        origin = self.headers.get("Origin")
        key = self.headers.get("Sec-WebSocket-Key", "")
        try:
            key_ok = len(base64.b64decode(key, validate=True)) == 16
        except binascii.Error:
            key_ok = False
        problem = None
        if origin is not None and urlparse(origin).netloc != self.headers.get("Host"):
            problem = (403, "cross-origin request")  # a WebSocket is not covered by the browser's same-origin rules
        elif self.headers.get("Upgrade", "").lower() != "websocket" or self.headers.get("Sec-WebSocket-Version") != "13" \
                or not key_ok:
            problem = (400, "expected a WebSocket")
        if problem:
            self.close_connection = True
            return self._json(problem[0], {"error": problem[1]})

        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        accept = base64.b64encode(hashlib.sha1((key + WEBSOCKET_GUID).encode()).digest()).decode()
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        self.wfile.flush()
        self.close_connection = True
        sock = self.connection
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(5.0)  # a page that stops reading is dropped instead of blocking this thread forever
        queue = self.server.video.subscribe()
        try:
            while not queue.closed:
                message = queue.get(timeout=0.25)
                if message is not None:
                    sock.sendall(websocket_frame(message[4:]))
                if select.select([sock], [], [], 0)[0] and not self._websocket_receive():
                    break
        except OSError:
            pass  # the page went away
        finally:
            self.server.video.unsubscribe(queue)

    def _websocket_receive(self):
        """Reads a frame from the page, which only sends control frames. False once the connection is over."""
        head = self.rfile.read(2)
        if len(head) < 2:
            return False
        opcode, length = head[0] & 0x0F, head[1] & 0x7F
        if length > 125:
            return False  # control frames are short; anything else is not expected
        mask = self.rfile.read(4) if head[1] & 0x80 else bytes(4)
        payload = bytes(b ^ mask[i % 4] for i, b in enumerate(self.rfile.read(length)))
        if opcode == 0x8:  # close: answer with the same code
            self.connection.sendall(websocket_frame(payload[:2], 0x8))
            return False
        if opcode == 0x9:  # ping
            self.connection.sendall(websocket_frame(payload, 0xA))
        return True

    def do_POST(self):
        if urlparse(self.path).path != "/api/command":
            self.close_connection = True
            return self._json(404, {"error": "not found"})
        # A page on another site can post to 127.0.0.1 too. Requiring JSON makes the browser ask first (a preflight that
        # this server never answers), and the Origin check covers the rest.
        origin = self.headers.get("Origin")
        problem = None
        if not self._host_ok():
            problem = (403, "unexpected Host header")
        elif origin is not None and urlparse(origin).netloc != self.headers.get("Host"):
            problem = (403, "cross-origin request")
        elif not self.headers.get("Content-Type", "").startswith("application/json"):
            problem = (415, "send application/json")
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            length = 0
        if problem is None and not 0 < length <= MAX_BODY:
            problem = (400, "bad body size")
        if problem:
            self.close_connection = True  # the body was not read
            return self._json(problem[0], {"error": problem[1]})

        command = protocol.decode_command(self.rfile.read(length))
        if command is None:
            return self._json(400, {"error": "not a command"})
        sent = self.server.link.send(protocol.encode_command(*command))
        self._json(200 if sent else 502, {"ok": sent})


def make_server(host="127.0.0.1", port=8000, robot_host="127.0.0.1", command_port=protocol.COMMAND_PORT,
                status_port=protocol.STATUS_PORT, status_bind="127.0.0.1", video_port=protocol.VIDEO_PORT):
    """(HTTP server, RobotLink). port=0 / status_port=0 pick free ports. The VideoRelay is httpd.video."""
    link = RobotLink(robot_host, command_port, status_port, status_bind)
    try:
        httpd = ThreadingHTTPServer((host, port), Handler)
    except OSError:
        link.close()
        raise
    httpd.daemon_threads = True
    httpd.link = link
    httpd.video = VideoRelay(robot_host, video_port)
    httpd.allowed_hosts = None if host in ("0.0.0.0", "") else {"localhost", "127.0.0.1", "[::1]", host}
    return httpd, link


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1", help="address to serve on (default: this computer only)")
    parser.add_argument("--port", type=int, default=8000, help="web port")
    parser.add_argument("--robot-host", default="127.0.0.1", help="where the robot controller runs")
    parser.add_argument("--status-bind", default="127.0.0.1",
                        help="address to receive the robot's status on; use 0.0.0.0 if the robot is on another computer")
    parser.add_argument("--no-browser", action="store_true", help="do not open the page")
    args = parser.parse_args()

    try:
        httpd, link = make_server(args.host, args.port, args.robot_host, status_bind=args.status_bind)
    except OSError as error:
        sys.exit(f"Cannot start: {error}. Another server may be running; try --port {args.port + 1}.")

    shown = "127.0.0.1" if args.host in ("0.0.0.0", "") else args.host
    url = f"http://{shown}:{httpd.server_port}/"
    print(f"Teleoperation page: {url}\n"
          f"Commands go to the robot at {args.robot_host}:{protocol.COMMAND_PORT}, status comes back on UDP {link.status_port}, "
          f"the camera from TCP {protocol.VIDEO_PORT}.\n"
          f"Open Webots/worlds/MOSAC_REAL_V4_teleop.wbt for the robot. Ctrl+C stops the server (and the robot).", flush=True)
    if not args.no_browser:
        threading.Timer(0.5, webbrowser.open, [url]).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):  # leave the robot stopped, whatever the page was doing
            link.send(protocol.encode_command("stop"))
        httpd.server_close()
        httpd.video.close()
        link.close()


if __name__ == "__main__":
    main()
