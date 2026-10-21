"""Teleoperation website: a page with the robot's camera and drive controls, hosted from here, that sends the commands to the
Webots robot.

    python server.py                    serves port 8000 on this computer and the local network, and opens it in the browser
    python server.py --port 8001 --no-browser
    python server.py --host 127.0.0.1   this computer only

The page is the React app in web_app_teleoperation (pnpm build puts it in dist/index.html, a single file). The page talks to this
server over WebRTC (webrtc.py): the camera is a video track and the commands and the robot's state go over a data channel; the
only HTTP besides the page is the POST that sets that connection up. The robot side is the Webots world MOSAC_REAL_V4_teleop.wbt
(controller Programmed_Yaw with --teleop). Commands go to it over UDP (see protocol.py); it reports its heading back, which the
page shows. While a page shows the video, the server also connects to the robot's camera (TCP) and passes the H.264 frames on.

Needs aiortc (pip install -r requirements.txt) for the WebRTC; the rest is the standard library.

There is no login: anyone on the local network can open the page and drive the robot (--host 127.0.0.1 keeps it to this computer).
"""
import argparse
import ipaddress
import json
import os
import socket
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import protocol  # noqa: E402

try:
    import webrtc  # noqa: E402
except ImportError as error:
    sys.exit(f"{error}\nThe page's WebRTC needs aiortc: pip install -r {os.path.join(HERE, 'requirements.txt')} "
             f"(in a virtual environment: python3 -m venv .venv)")

PAGE = os.path.join(HERE, "web_app_teleoperation", "dist", "index.html")
ICONS = ("/icon-192.png", "/icon-512.png", "/apple-touch-icon.png")  # web_app_teleoperation/public, copied next to the page by the build
# The page as an installed app (Android: Install app, which needs https or localhost; iPhone: Share, Add to Home Screen): it starts
# full screen, with no browser bars, which is the only full screen a phone's browser allows.
MANIFEST = {
    "name": "MOSAC Teleoperation", "short_name": "MOSAC Teleop", "id": "/", "start_url": "/", "scope": "/",
    "display": "fullscreen", "display_override": ["fullscreen", "standalone"],
    "background_color": "#000000", "theme_color": "#000000",
    "icons": [{"src": "/icon-192.png", "sizes": "192x192", "type": "image/png", "purpose": "any"},
              {"src": "/icon-512.png", "sizes": "512x512", "type": "image/png", "purpose": "any"},
              {"src": "/icon-512.png", "sizes": "512x512", "type": "image/png", "purpose": "maskable"}],
}
MAX_BODY = 64 << 10  # bytes of a connection offer (an SDP with candidates is a few kB)
STALE_AFTER = 1.0    # s without a status message before the robot counts as disconnected
VIDEO_LINGER = 3.0   # s the camera stays connected after the last page leaves (a reload does not restart the encoder)


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
    """The camera: connected to the robot only while some page shows the video, it hands each message to every page's sink."""

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

    def subscribe(self, sink):
        """sink (put(message) and close(), like protocol.VideoQueue) gets the video until unsubscribe(sink)."""
        with self._lock:
            self._viewers.append(sink)
            if self._config is not None:
                sink.put(self._config)
        self._wake.set()

    def unsubscribe(self, sink):
        sink.close()
        with self._lock:
            if sink in self._viewers:
                self._viewers.remove(sink)

    def state(self):
        with self._lock:
            live = self._last_frame is not None and time.monotonic() - self._last_frame < STALE_AFTER
            return {"live": live, "viewers": len(self._viewers)}

    def close(self):
        self._closed = True
        self._wake.set()
        with self._lock:
            viewers, self._viewers = self._viewers, []
        for sink in viewers:
            sink.close()
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
                    for sink in viewers:
                        sink.put(message)
            if self._watched():
                alone_since = None
            elif alone_since is None:
                alone_since = time.monotonic()
            elif time.monotonic() - alone_since > self.linger:
                return  # the robot switches its camera off when this connection closes


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        # A phone that leaves the network, closes the tab or reloads drops its connections, between two requests or in the middle
        # of one (the page is 250 kB). The browser's end going away is not an error of ours; the default prints a traceback each time.
        if isinstance(sys.exc_info()[1], ConnectionError):
            return
        super().handle_error(request, client_address)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "MOSACTeleop"

    def log_message(self, format, *args):
        pass

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
        host = self.headers.get("Host", "")
        name = host if host.endswith("]") or ":" not in host else host.rsplit(":", 1)[0]
        name = name.lower()
        if name in self.server.allowed_hosts:
            return True
        # On the network the page is reached by whatever address the other device has for this computer. A DNS rebinding
        # attack needs a name of its own, so an address in numbers is safe.
        if self.server.lan:
            try:
                ipaddress.ip_address(name.strip("[]"))
            except ValueError:
                return False
            return True
        return False

    def do_GET(self):
        if not self._host_ok():
            self.close_connection = True
            return self._json(403, {"error": "unexpected Host header"})
        path = urlparse(self.path).path
        if path == "/manifest.webmanifest":
            return self._send(200, json.dumps(MANIFEST).encode(), "application/manifest+json")
        if path in ICONS:
            try:
                with open(os.path.join(os.path.dirname(PAGE), path[1:]), "rb") as f:
                    return self._send(200, f.read(), "image/png")
            except FileNotFoundError:
                return self._json(404, {"error": "not built"})
        if path not in ("/", "/index.html"):
            return self._json(404, {"error": "not found"})
        try:
            with open(PAGE, "rb") as f:
                self._send(200, f.read(), "text/html; charset=utf-8")
        except FileNotFoundError:
            self._send(503, b"The page is not built yet: cd web_app_teleoperation && pnpm install && pnpm build\n",
                       "text/plain; charset=utf-8")

    def do_POST(self):
        if urlparse(self.path).path != "/api/offer":
            self.close_connection = True
            return self._json(404, {"error": "not found"})
        # A page on another site can post to 127.0.0.1 too, and the connection it would get drives the robot. Requiring JSON
        # makes the browser ask first (a preflight that this server never answers), and the Origin check covers the rest.
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

        try:
            offer = json.loads(self.rfile.read(length))
            sdp, kind = offer["sdp"], offer["type"]
            if not isinstance(sdp, str) or kind != "offer":
                raise ValueError
        except (ValueError, KeyError, TypeError):
            return self._json(400, {"error": "not an offer"})
        try:
            answer = self.server.pages.answer(sdp, kind)
        except webrtc.Busy:
            return self._json(503, {"error": "too many pages connected"})
        except Exception as error:  # aiortc rejects an SDP it cannot use with a ValueError, or something less specific
            return self._json(400, {"error": f"cannot connect: {error}"})
        self._json(200, answer)


def make_server(host="127.0.0.1", port=8000, robot_host="127.0.0.1", command_port=protocol.COMMAND_PORT,
                status_port=protocol.STATUS_PORT, status_bind="127.0.0.1", video_port=protocol.VIDEO_PORT):
    """(HTTP server, RobotLink). port=0 / status_port=0 pick free ports. The VideoRelay is httpd.video, the connected pages
    are httpd.pages."""
    link = RobotLink(robot_host, command_port, status_port, status_bind)
    try:
        httpd = Server((host, port), Handler)
    except OSError:
        link.close()
        raise
    httpd.link = link
    httpd.video = VideoRelay(robot_host, video_port)
    httpd.pages = webrtc.Pages(link, httpd.video)
    httpd.lan = host in ("0.0.0.0", "")
    httpd.allowed_hosts = {"localhost", "127.0.0.1", "[::1]", host.lower()}
    if httpd.lan:
        computer = socket.gethostname().lower()
        httpd.allowed_hosts |= {computer, computer.split(".")[0] + ".local"}
    return httpd, link


def lan_address():
    """This computer's address on the local network (the one its default route uses), or None if it has none. Sends nothing."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.connect(("10.255.255.255", 9))
            address = probe.getsockname()[0]
        except OSError:
            return None
    return None if address.startswith("127.") or address == "0.0.0.0" else address


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="0.0.0.0",
                        help="address to serve on (default: every network interface; 127.0.0.1 for this computer only)")
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

    shown = "127.0.0.1" if httpd.lan else args.host
    url = f"http://{shown}:{httpd.server_port}/"
    network = lan_address() if httpd.lan else None
    on_network = (f"On the local network: http://{network}:{httpd.server_port}/ (no login: anyone there can drive the robot)\n"
                  if network else "")
    print(f"Teleoperation page: {url}\n{on_network}"
          f"Commands go to the robot at {args.robot_host}:{protocol.COMMAND_PORT}, status comes back on UDP {link.status_port}, "
          f"the camera from TCP {protocol.VIDEO_PORT}.\n"
          f"Open Webots/worlds/MOSAC_REAL_V4_teleop.wbt for the robot. Ctrl+C stops the server (and the robot).", flush=True)
    if not os.path.isfile(PAGE):
        print(f"The page is not built yet: cd {os.path.join(HERE, 'web_app_teleoperation')} && pnpm install && pnpm build",
              file=sys.stderr, flush=True)
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
        httpd.pages.close()
        httpd.video.close()
        link.close()


if __name__ == "__main__":
    main()
