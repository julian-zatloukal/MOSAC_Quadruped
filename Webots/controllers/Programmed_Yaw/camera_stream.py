"""The robot's camera for the Teleoperation page: H.264 at 30 fps, encoded by ffmpeg and served over TCP. Pure python (no Webots).

CameraStream listens on protocol.VIDEO_PORT. While the web server is connected it runs ffmpeg (libx264, low latency settings),
feeds it the frames given to push() and sends each encoded frame as a video message (../../../Teleoperation/protocol.py). With
nobody connected there is no encoder and wanted is False, so the controller does not even render the camera. CameraCapture does
that part for a Webots camera: it renders it at 30 fps on the 10 ms control step and pushes the images.

ffmpeg is not part of Python: brew install ffmpeg (macOS), apt install ffmpeg (Linux). Without it there is no video; driving the
robot works the same. TELEOP_FFMPEG=<path> points to another ffmpeg.
"""
import base64
import collections
import json
import math
import os
import select
import shutil
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "Teleoperation"))

import protocol  # noqa: E402

FPS = 30
# A live view: no B-frames and no look-ahead (zerolatency), a key frame every second so that a page can join quickly, and a
# quality target (crf) capped for Wi-Fi. Measured with the 1280x720 camera panning and bobbing like the walking robot: 7 ms from
# frame in to packet out, 2.7 Mbit/s, 60% of one M4 core. (h264_videotoolbox used a third of the CPU but held 170 ms of frames.)
ENCODER_ARGS = ["-c:v", "libx264", "-preset", "veryfast", "-tune", "zerolatency", "-crf", "23", "-maxrate", "4M",
                "-bufsize", "2M", "-g", str(FPS), "-pix_fmt", "yuv420p"]


def find_ffmpeg():
    """Path of ffmpeg, or None. Also looks where Homebrew installs it: Webots started from the Dock does not get the shell's
    PATH."""
    for candidate in (os.environ.get("TELEOP_FFMPEG"), shutil.which("ffmpeg"), "/opt/homebrew/bin/ffmpeg",
                      "/usr/local/bin/ffmpeg", "/usr/bin/ffmpeg"):
        if candidate and os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return None


def codec_string(avcc):
    """The codec name browsers want ("avc1.64001f") from an H.264 decoder configuration record."""
    return "avc1." + avcc[1:4].hex()


class FlvReader:
    """The H.264 packets in ffmpeg's FLV output. FLV is used only because it gives each frame's size, so a frame can be sent
    as soon as it is encoded (in a raw H.264 stream its end shows only when the next frame starts)."""

    def __init__(self):
        self._buffer = bytearray()
        self._started = False

    def feed(self, data):
        """Adds bytes. Returns the packets they completed: (protocol.VIDEO_CONFIG, decoder configuration record) or
        (protocol.VIDEO_KEY / VIDEO_DELTA, the frame's NAL units with 4-byte lengths)."""
        self._buffer += data
        packets = []
        if not self._started:
            if len(self._buffer) < 9:
                return packets
            if self._buffer[:3] != b"FLV":
                raise ValueError("ffmpeg did not write FLV")
            skip = int.from_bytes(self._buffer[5:9], "big") + 4  # the header, and the size of the tag before the first
            if len(self._buffer) < skip:
                return packets
            del self._buffer[:skip]
            self._started = True
        while len(self._buffer) >= 11:
            size = int.from_bytes(self._buffer[1:4], "big")
            if len(self._buffer) < 11 + size + 4:
                break
            tag_type, data = self._buffer[0] & 0x1F, bytes(self._buffer[11:11 + size])
            del self._buffer[:11 + size + 4]
            if tag_type != 9 or len(data) < 5:  # not video (metadata)
                continue
            if data[0] & 0x80 or data[0] & 0x0F != 7:
                raise ValueError("FLV video that is not H.264 in the classic FLV format")
            if data[1] == 0:
                packets.append((protocol.VIDEO_CONFIG, data[5:]))
            elif data[1] == 1:
                packets.append((protocol.VIDEO_KEY if data[0] >> 4 == 1 else protocol.VIDEO_DELTA, data[5:]))
        return packets


class _Encoder:
    """One ffmpeg process: frames go in with push(), packets come out through on_packet(encoder, kind, capture time, payload),
    and on_exit(encoder) is called when ffmpeg ends (after stop(), or if it fails)."""

    def __init__(self, ffmpeg, width, height, fps, on_packet, on_exit, log):
        command = [ffmpeg, "-hide_banner", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgra", "-s", f"{width}x{height}",
                   "-framerate", str(fps), "-i", "pipe:0", "-an", *ENCODER_ARGS, "-fps_mode", "passthrough",
                   "-flush_packets", "1", "-flvflags", "no_duration_filesize+no_metadata", "-f", "flv", "pipe:1"]
        self._process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                         bufsize=0)
        self._on_packet, self._on_exit, self._log = on_packet, on_exit, log
        self._cond = threading.Condition()
        self._frame = None        # (frame, capture time) waiting for the writer
        self._stamps = collections.deque()  # capture times of the frames inside ffmpeg, oldest first
        self._stopping = False
        self.skipped = 0          # frames replaced before ffmpeg took them
        for target in (self._write, self._read, self._errors):
            threading.Thread(target=target, daemon=True).start()

    def push(self, frame, stamp):
        with self._cond:
            if self._frame is not None:
                self.skipped += 1
            self._frame = (frame, stamp)
            self._cond.notify()

    def stop(self):
        """Ends the stream and waits for ffmpeg (up to 2 s, then kills it)."""
        with self._cond:
            self._stopping = True
            self._cond.notify()
        try:
            self._process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self._process.kill()
            self._process.wait()

    def _write(self):
        stdin = self._process.stdin
        try:
            while True:
                with self._cond:
                    while self._frame is None and not self._stopping:
                        self._cond.wait()
                    if self._stopping:
                        break
                    (frame, stamp), self._frame = self._frame, None
                self._stamps.append(stamp)
                view = memoryview(frame)
                while view:
                    view = view[stdin.write(view):]
        except (OSError, ValueError):
            pass  # ffmpeg is gone; _read reports it
        finally:
            try:
                stdin.close()
            except OSError:
                pass

    def _read(self):
        reader = FlvReader()
        try:
            while True:
                data = self._process.stdout.read(1 << 16)
                if not data:
                    break
                for kind, payload in reader.feed(data):
                    if kind == protocol.VIDEO_CONFIG or not self._stamps:
                        stamp = time.time_ns() // 1000
                    else:
                        stamp = self._stamps.popleft()  # one packet per frame, in order: no B-frames, no dropped frames
                    self._on_packet(self, kind, stamp, payload)
        except ValueError as error:
            self._log(f"[video] {error}")
            self._process.kill()
        finally:
            self._process.wait()
            self._on_exit(self)

    def _errors(self):
        for line in self._process.stderr:
            self._log(f"[video] ffmpeg: {line.decode(errors='replace').rstrip()}")


class _Client:
    """A connected web server: its own queue and thread, so that a slow one never holds up the encoder or the others."""

    def __init__(self, sock, on_close):
        self.queue = protocol.VideoQueue()
        self._sock = sock
        self._on_close = on_close
        threading.Thread(target=self._send, daemon=True).start()

    def close(self):
        self.queue.close()

    def _send(self):
        try:
            while not self.queue.closed:
                message = self.queue.get(timeout=0.5)
                if message is not None:
                    self._sock.sendall(message)
                elif select.select([self._sock], [], [], 0)[0] and not self._sock.recv(1024):
                    break  # the server hung up (it never sends anything)
        except OSError:
            pass
        finally:
            self.queue.close()
            self._sock.close()
            self._on_close(self)


class CameraStream:
    def __init__(self, width, height, fps=FPS, port=protocol.VIDEO_PORT, bind_host="127.0.0.1", ffmpeg=None, log=print):
        """width, height: of the frames that push() will get (BGRA, 4 bytes a pixel, as Webots gives them). port, bind_host:
        where the web server connects. ffmpeg: its path (default: find_ffmpeg()). log: called with a line of text."""
        self.width, self.height, self.fps = width, height, fps
        self._ffmpeg = ffmpeg or find_ffmpeg()
        if self._ffmpeg is None:
            raise FileNotFoundError("ffmpeg not found (brew install ffmpeg, or set TELEOP_FFMPEG)")
        self._log = log
        self._lock = threading.Lock()
        self._clients = []
        self._encoder = None
        self._config = None  # the config message of the running encoder, for servers that connect later
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._listener.bind((bind_host, port))
        except OSError as error:
            self._listener.close()
            raise OSError(f"cannot serve the camera on TCP {bind_host}:{port} ({error})") from error
        self._listener.listen(4)
        self.port = self._listener.getsockname()[1]
        self._closed = False
        threading.Thread(target=self._accept, daemon=True).start()

    @property
    def wanted(self):
        """True while a server is connected: frames are worth capturing."""
        return self._encoder is not None

    def push(self, frame, timestamp_us=None):
        """One frame (bytes, BGRA). Never blocks: if ffmpeg has not taken the previous frame yet, this one replaces it."""
        encoder = self._encoder
        if encoder is not None:
            encoder.push(frame, timestamp_us if timestamp_us is not None else time.time_ns() // 1000)

    def close(self):
        self._closed = True
        self._listener.close()
        with self._lock:
            clients, self._clients = self._clients, []
            encoder, self._encoder = self._encoder, None
        for client in clients:
            client.close()
        if encoder is not None:
            encoder.stop()

    def _accept(self):
        while not self._closed:
            try:
                sock, address = self._listener.accept()
            except OSError:
                return
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(5.0)  # a server that stops reading is dropped instead of blocking its thread forever
            with self._lock:
                client = _Client(sock, self._remove)
                self._clients.append(client)
                if self._config is not None:
                    client.queue.put(self._config)
                if self._encoder is None:
                    self._config = None
                    self._encoder = _Encoder(self._ffmpeg, self.width, self.height, self.fps, self._on_packet,
                                             self._on_exit, self._log)
            self._log(f"[video] sending the camera to {address[0]}")

    def _on_packet(self, encoder, kind, stamp, payload):
        if kind == protocol.VIDEO_CONFIG:
            payload = json.dumps({"codec": codec_string(payload), "width": self.width, "height": self.height,
                                  "fps": self.fps, "avcc": base64.b64encode(payload).decode()}).encode()
        message = protocol.encode_video(kind, stamp, payload)
        with self._lock:
            if encoder is not self._encoder:
                return  # a stopped encoder's last packets
            if kind == protocol.VIDEO_CONFIG:
                self._config = message
            clients = list(self._clients)
        for client in clients:
            client.queue.put(message)

    def _on_exit(self, encoder):
        with self._lock:
            if encoder is not self._encoder:
                return  # stopped on purpose
            self._encoder = self._config = None
            clients, self._clients = self._clients, []
        self._log("[video] ffmpeg stopped; the stream restarts when the server reconnects")
        for client in clients:
            client.close()

    def _remove(self, client):
        with self._lock:
            if client in self._clients:
                self._clients.remove(client)
            encoder = None
            if not self._clients and self._encoder is not None:
                encoder, self._encoder, self._config = self._encoder, None, None
        if encoder is not None:
            self._log("[video] nobody watching, camera off")
            encoder.stop()


class CameraCapture:
    """Renders a Webots camera at the stream's frame rate while the stream is wanted, and pushes the images. update() once per
    control step.

    Webots renders a camera only on a control step, the first one after its sampling period has passed: a 33 ms period gives
    25 fps on a 10 ms step. So the period is set again after every image, to reach the step closest after the next 1/30 s:
    40, 30, 30, 40... ms, which is 30 fps on average, and nothing is rendered in between."""

    def __init__(self, camera, stream, step_ms):
        """camera: a Webots Camera (enable, disable, getImage). stream: a CameraStream. step_ms: the control step."""
        self.camera, self.stream, self.step_ms = camera, stream, step_ms
        self._start = None  # simulated time (ms) when the camera was switched on; None while it is off
        self._next = 0      # the number of the next frame

    def _due(self, k):
        return self._start + math.ceil(k * 1000 / self.stream.fps / self.step_ms - 1e-9) * self.step_ms

    def update(self, now_ms):
        """now_ms: the simulated time, in ms."""
        if not self.stream.wanted:
            if self._start is not None:
                self.camera.disable()
                self._start = None
            return
        if self._start is None:
            self._start, self._next = now_ms, 1
            self.camera.enable(self._due(1) - now_ms)
            return
        if now_ms >= self._due(self._next):
            self.stream.push(self.camera.getImage())
            while self._due(self._next) <= now_ms:
                self._next += 1
            self.camera.enable(self._due(self._next) - now_ms)

    def close(self):
        self.stream.close()


def start_camera(camera, step_ms, bind_host="127.0.0.1", log=print):
    """A CameraCapture streaming a Webots camera, or None when it cannot (the reason goes to log)."""
    try:
        stream = CameraStream(camera.getWidth(), camera.getHeight(), bind_host=bind_host, log=log)
    except (OSError, FileNotFoundError) as error:
        log(f"[video] no camera stream: {error}")
        return None
    log(f"[video] camera {stream.width}x{stream.height} at {stream.fps} fps, H.264 on TCP {stream.port} while the page shows it")
    return CameraCapture(camera, stream, step_ms)
