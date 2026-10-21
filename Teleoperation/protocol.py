"""Messages between the Teleoperation web server and the robot (the Webots controller Programmed_Yaw --teleop), and between the
server and the page. Imported by both ends of the robot link, so the ports and the timeout live in one place.

Commands and status are UDP, one JSON object per datagram:

  server -> robot, to COMMAND_PORT:
      {"cmd": "drive", "forward": 1.0, "turn": -0.5}
                  walk at this fraction of the full stride (-1..1, negative = backwards) while turning at this fraction of the
                  full turn (-1..1, positive = to the left). With forward 0 it turns on the spot, with both 0 it stops.
      {"cmd": "stop"}                halt
  A drive only lasts COMMAND_TIMEOUT seconds: the sender has to repeat it while the button is held, and the robot stops by itself
  when the repeats stop (page closed, network lost, server killed).

  robot -> server, to STATUS_PORT, every STATUS_PERIOD seconds:
      {"yaw_deg", "x_m", "y_m", "mode", "idle", "forward", "rate_deg_s", "max_rate_deg_s"}   see remote.RemoteControl.status()

The camera goes over TCP instead (camera_stream.py on the robot side): the web server connects to VIDEO_PORT while a page shows
the video, and the robot sends video messages until it disconnects. Each message is a 4-byte length (big-endian, the bytes that
follow), the kind (1 byte), the capture time (8 bytes, microseconds since the epoch) and the payload:
      VIDEO_CONFIG  {"codec": "avc1.42c01f", "width": 1280, "height": 720, "fps": 30, "avcc": base64}, JSON. First, and again
                    whenever the encoder restarts. avcc is the H.264 decoder configuration (SPS and PPS, ISO 14496-15).
      VIDEO_KEY     one H.264 frame that decodes on its own (IDR), as NAL units with 4-byte lengths (AVCC)
      VIDEO_DELTA   one H.264 frame that needs the frames before it, same format

The page talks to the web server over WebRTC (webrtc.py). The camera is a video track: the frames above as they are, with the
parameter sets in front of each key frame. The data channel "teleop" carries, as JSON text messages,
  page -> server:   the commands above ({"cmd": "drive", ...} / {"cmd": "stop"}), repeated while a button is held
  server -> page:   every STATUS_PERIOD seconds {"connected": bool, "age": s since the robot's last status or null,
                    "robot": the status above or null, "video": {"live": bool, "viewers": n}}
"""
import collections
import json
import math
import struct
import threading

COMMAND_PORT = 9871    # the robot listens here
STATUS_PORT = 9872     # the web server listens here
VIDEO_PORT = 9873      # the robot serves its camera here (TCP)
COMMAND_TIMEOUT = 0.5  # s without a drive command and the robot stops
STATUS_PERIOD = 0.1    # s between status messages

VIDEO_CONFIG, VIDEO_KEY, VIDEO_DELTA = 0, 1, 2
VIDEO_HEADER = struct.Struct(">IBQ")  # length, kind, capture time
VIDEO_MAX = 4 << 20                   # bytes; a 720p key frame is ~70 kB


def encode_command(cmd, forward=0.0, turn=0.0):
    if cmd == "drive":
        return json.dumps({"cmd": "drive", "forward": forward, "turn": turn}).encode()
    if cmd == "stop":
        return json.dumps({"cmd": "stop"}).encode()
    raise ValueError(f"unknown command {cmd!r}")


def _fraction(value):
    """value clamped to [-1, 1], or None if it is not a finite number."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None
    return max(-1.0, min(1.0, float(value)))


def decode_command(data):
    """("drive", forward, turn) with forward and turn clamped to [-1, 1], ("stop", 0.0, 0.0), or None for anything that is not
    a valid command. A missing forward or turn counts as 0."""
    try:
        message = json.loads(data)
    except ValueError:
        return None
    if not isinstance(message, dict):
        return None
    if message.get("cmd") == "stop":
        return ("stop", 0.0, 0.0)
    if message.get("cmd") == "drive":
        forward, turn = _fraction(message.get("forward", 0.0)), _fraction(message.get("turn", 0.0))
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


def encode_video(kind, timestamp_us, payload):
    return VIDEO_HEADER.pack(VIDEO_HEADER.size - 4 + len(payload), kind, timestamp_us) + payload


class VideoReader:
    """Cuts a TCP byte stream into whole video messages."""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data):
        """Adds what recv() returned. Returns the messages it completed (each with its length), oldest first. Raises ValueError
        when the stream is not made of video messages."""
        self._buffer += data
        messages = []
        while len(self._buffer) >= VIDEO_HEADER.size:
            length, kind, _ = VIDEO_HEADER.unpack_from(self._buffer)
            if not VIDEO_HEADER.size - 4 <= length <= VIDEO_MAX or kind > VIDEO_DELTA:
                raise ValueError(f"not a video message (length {length}, kind {kind})")
            if len(self._buffer) < 4 + length:
                break
            messages.append(bytes(self._buffer[:4 + length]))
            del self._buffer[:4 + length]
        return messages


class VideoQueue:
    """The video messages waiting to be sent to one receiver. A receiver more than `backlog` frames behind skips to the next key
    frame (the frames in between cannot be decoded without the ones dropped), so a slow page or network never builds up delay
    and never holds up the sender. A new queue starts at a key frame for the same reason."""

    def __init__(self, backlog=30):
        self.backlog = backlog
        self.closed = False
        self.skipped = 0  # frames dropped
        self._messages = collections.deque()
        self._frames = 0  # frames in _messages (the rest are config messages)
        self._need_key = True
        self._cond = threading.Condition()

    def put(self, message):
        """A whole message, with its length."""
        kind = message[4]
        with self._cond:
            if self.closed:
                return
            if kind != VIDEO_CONFIG:
                if self._frames >= self.backlog:
                    self.skipped += self._frames
                    self._messages = collections.deque(m for m in self._messages if m[4] == VIDEO_CONFIG)
                    self._frames = 0
                    self._need_key = True
                if kind == VIDEO_DELTA and self._need_key:
                    self.skipped += 1
                    return
                self._need_key = False
                self._frames += 1
            self._messages.append(message)
            self._cond.notify()

    def get(self, timeout=None):
        """The oldest message, or None if there is none after timeout seconds or the queue is closed."""
        with self._cond:
            if not self._messages and not self.closed:
                self._cond.wait(timeout)
            if not self._messages:
                return None
            message = self._messages.popleft()
            if message[4] != VIDEO_CONFIG:
                self._frames -= 1
            return message

    def close(self):
        with self._cond:
            self.closed = True
            self._cond.notify_all()
