# Teleoperation

A web page that shows the robot's camera full screen and walks the robot in Webots, forward and backwards, and turns it on
the spot. Made for a phone first (portrait or landscape), and it works the same on a desktop. The page is a React app
(`web_app_teleoperation/`, built to one HTML file) hosted by a Python script, and the video and the commands go between them
over WebRTC. The camera needs ffmpeg on the robot's side (`brew install ffmpeg`).

```
Once:   cd Teleoperation/web_app_teleoperation && pnpm install && pnpm build         the page, into dist/index.html
        cd Teleoperation && python3 -m venv .venv && .venv/bin/pip install -r requirements.txt     aiortc, for the WebRTC

1. Open Webots/worlds/MOSAC_REAL_V4_teleop.wbt in Webots (the robot stands up and waits). ./open_webots.sh does it in a new window.
2. .venv/bin/python server.py        opens http://127.0.0.1:8000 in the browser
```

| control | does |
|---|---|
| **▲** (hold, left thumb) | walk forward, holding the heading. Keys: `↑` `W` |
| **▼** (hold, left thumb) | walk backwards, holding the heading. Keys: `↓` `S` |
| **⟲** / **⟳** (hold, right thumb) | yaw left / right: turn on the spot, or walk a gentle curve if ▲ or ▼ is held too. Keys: `←` `A` / `→` `D` |
| top right | fill the screen with the video (cropped) or show the whole picture; full screen, where the browser allows it (not on iPhone) |

There is no stop button and no speed setting: the robot stops when everything is let go, and always moves at full speed (the
stride is 44.8 mm/s forward and 50.9 mm/s backwards, the turn 7.8 deg/s on the spot and 2.3 deg/s on a curve). Several buttons
can be held at once, like WASD; ▲ and ▼ together cancel out. The yaw buttons turn the robot's nose that way whichever
way it walks, so ▼ with ⟲ swings the nose to the left (counter-clockwise) and the tail to the right, unlike a car's steering
wheel when reversing. Walking forward, walking backwards and turning on the spot use three different gaits, so going from one
to another makes the robot stop and re-centre its legs first: 2 to 2.7 s.

The pill at the top left is a dot and a word: *Online* (green) when the page reaches the server and the server the robot,
*No robot* or *Connecting* (amber), *Offline* (red). Tap it for the details, and tap anywhere else to close them: the robot's
heading and what it is doing (Walking, Reversing, Turning, Stopping while the legs re-centre, Standing), and the quality of the
link, read once a second from the browser's WebRTC statistics (only while the details are open):

| | |
|---|---|
| Latency | round trip to the server |
| Bitrate, Frame rate, Resolution | of the video arriving |
| Loss, Jitter | of its packets (amber past 1% and 30 ms, red past 5% and 80 ms) |
| Video delay | in the browser: jitter buffer plus decoding |
| Freezes, Dropped, Retransmits | the picture stopping, frames given up on, packets asked for again: totals since connecting |
| the last line | the route: direct or relayed, UDP, the network type if the browser says |

## The camera

A cheap USB webcam on a stand at the front of the body's top plate (`camera` in `MOSAC_REAL_V4_teleop.wbt`): 1280x720, 63 deg
horizontal field of view, 22 cm above the ground and tilted 10 deg down, so the ground shows from about 0.4 m in front of
it. It has no mass and no collision
shape, so the tuned gaits are not affected. The robot side (`Webots/controllers/Programmed_Yaw/camera_stream.py`) renders it
at 30 fps and encodes H.264 with ffmpeg (libx264 `veryfast`, `zerolatency`, Constrained Baseline, a key frame every second).
Measured on the M4 with the view moving like the walking robot's (High profile): 7 ms to encode a frame, 2.7 Mbit/s, 60% of
one core, and the camera costs Webots about 2.5 ms per frame. Nothing is rendered or encoded while no page shows the video, and
the camera switches off 3 s after the last page leaves.

The server does not decode or encode the video again: aiortc takes the H.264 frames as the robot's encoder made them and cuts
them into RTP packets, so the browser plays them with its own decoder (hardware, on most phones) and the picture costs the
server almost nothing. The choices, for a live view where delay matters more than the last bit of compression:

- **Constrained Baseline**, no B-frames: what WebRTC browsers negotiate, nothing to reorder before showing a frame, cheaper to
  decode on a phone and a little cheaper to encode. The price is 10-15% more bitrate than High at the same quality (measured on
  synthetic test pictures, not the Webots camera); remove `-profile:v baseline` from `camera_stream.py` to trade it back.
- **A rate buffer of 1 Mbit** (`-bufsize`, was 2) with a cap of 4 Mbit/s: the server sends a frame in one burst, without the
  pacing a browser's own sender does, so a key frame is kept to about 1 Mbit.
- **A key frame every second.** A browser that loses a packet asks for one (PLI), but that request cannot reach the encoder
  through the server, so the picture recovers within a second instead. A page that opens while the camera is already on
  waits at most a second for its first picture, too. Lost packets are retransmitted first (NACK), which is why a glitch on a
  good network does not even show.
- Nothing to adapt to the network: the bitrate is fixed, so on a weak Wi-Fi the picture breaks up instead of getting blurrier.
  Lower `-maxrate` in `camera_stream.py` if that happens.

Measured in Chrome on the same computer, with the real camera code and ffmpeg fed synthetic 720p frames (Webots not involved):
the first picture 0.4 s after the page opens, 30 fps, and 30-50 ms from the frame being made to it being on the screen (p95 below
65 ms), of which about 20 ms is the browser's. The earlier WebSocket page measured 8 ms (WebCodecs) and 65 ms (Media Source);
this is the price of a path that also works on a phone over plain http, and with nothing for the page to decode itself.

## How it works

```
browser --WebRTC data channel "teleop": drive/stop, every 100 ms while held--> server.py --UDP 9871--> Webots: Programmed_Yaw --teleop
browser <--WebRTC data channel: the robot's state, 10 times a second---------- server.py <--UDP 9872-- (heading, position, state, every 100 ms)
browser <--WebRTC video track: the camera's H.264, as it is------------------- server.py <--TCP 9873-- camera_stream.py (while watched)
browser --POST /api/offer once, to set the connection up--> server.py       (the only HTTP besides the page itself)
```

The command is `{"cmd": "drive", "forward": -1..1, "turn": -1..1}` (negative forward = backwards) or `{"cmd": "stop"}`
(`protocol.py`, shared by both sides, also describes the video messages and what the data channel carries). The channel is
ordered but drops a command that could not be delivered in 200 ms instead of retransmitting it for as long as it takes: the
next one is already on its way, and a stale one would hold it up. On the Webots side `Programmed_Yaw.py --teleop`
hands the commands to a `MotionController` (`Webots/controllers/Programmed_Yaw/remote.py` and `motion_core.py`), so stopping,
reversing, the speed ramps and walking behave as described in that README. The server only passes the video on: it connects to
the camera while some page shows it, and a page that falls behind skips to the next key frame instead of building up delay.
No STUN or TURN servers are used, so nothing leaves the computer or its network.

## The page

`web_app_teleoperation/` is a Vite + React + TypeScript app (React Compiler on, `vite-plugin-singlefile`: the build is the single
file `dist/index.html`, which `server.py` serves). `pnpm build` builds it, `pnpm lint` checks it. For working on it,
`pnpm dev` serves it with hot reload on http://localhost:5173 and sends the connection setup to the server
(`python3 server.py --no-browser`, on port 8000); the video and commands still go straight to the server over WebRTC.
`src/teleop/` is the plain TypeScript (the messages, the WebRTC connection, turning held buttons into commands), `src/hooks/`
and `src/components/` the React side.

## Safety

- **The robot stops by itself.** A drive command lasts 0.5 s, so the page has to keep repeating it. If the page is closed, the
  network drops, or the server is killed, the robot stops within 0.5 s (measured: 0.55 s). The server also
  sends a stop when it is shut down with Ctrl+C.
- **The page lets go on its own** when the button, key or touch is released, the window loses focus, or the tab is hidden.
- **Only this computer can reach the page** (`127.0.0.1`). There is no login, so `--host 0.0.0.0` lets anything on the network
  drive the robot and watch its camera; do not use it on a network you do not trust. The request that sets up the connection
  must be JSON, from the page's own origin, to the server's own host name, which keeps other websites open in your browser from
  steering the robot or reading the camera. The connection's own UDP ports are not limited to this computer even then (the
  browser connects to the computer's own address), but they only answer someone who has the secrets from that request.

## Options

```
python3 server.py --host 0.0.0.0                                      for a phone on the same network: open http://<this computer's IP>:8000
python3 server.py --port 8001 --no-browser
python3 server.py --robot-host 192.168.1.50 --status-bind 0.0.0.0     the robot's controller is on another computer
```

On the robot's side `TELEOP_BIND` (where it listens for commands and camera connections, default `127.0.0.1`),
`TELEOP_SERVER_HOST` (where it sends the status) and `TELEOP_FFMPEG` (the ffmpeg to use; by default it looks in the PATH and
where Homebrew installs it, since Webots started from the Dock does not get the shell's PATH) are read from the environment of
Webots. The ports are at the top of `protocol.py`; the encoder settings at the top of `camera_stream.py`; the resolution and the
field of view in the world's `camera` node.

## Files and tests

- `server.py` the web server (the page and `/api/offer`), the UDP link and the video relay. `webrtc.py` the WebRTC side: one
  connection per page, the camera as a video track, the data channel. `protocol.py` the messages.
  `web_app_teleoperation/` the page. `requirements.txt` aiortc, the only thing beyond the standard library.
- `python3 test_teleop.py` (needs aiortc) runs the page's server against the real robot-side code over loopback, with a Python
  WebRTC client in place of the browser, without Webots: the state, commands, the timeout, reversals, and the requests the
  server must refuse. It does not cover the video.

## Troubleshooting

- *No robot* on the page: the world is not open, the simulation is paused, or the UDP ports are taken (Webots' console says so).
  Only one `MOSAC_REAL_V4_teleop.wbt` can run at a time.
- *No video* with the robot connected: Webots' console says why after `[video]`, usually that ffmpeg was not found.
- *Offline* while the server is running: the page and the server cannot set up the WebRTC connection (it retries by itself). A
  firewall that blocks incoming UDP for Python, or a VPN, does it. The server's log shows nothing; `chrome://webrtc-internals`
  does.
- *Video not supported*: the browser has no WebRTC with H.264.
- *The page is not built yet* (in the browser or when the server starts): `cd web_app_teleoperation && pnpm install && pnpm build`.
- *Cannot start ... try --port*: something else is on port 8000.
- *The WebRTC needs aiortc*: `pip install -r requirements.txt`, in a virtual environment (`python3 -m venv .venv`).

## Not done yet

No walking sideways, and `TurnController.go_to(heading)` exists but is not in the protocol or the page. The
receiver is the Webots controller; the real robot has no counterpart yet (neither for the commands nor for a camera).
Checked in Chrome (desktop, and phone screen sizes) only, not in Safari or Firefox or on a real phone, and the camera with
synthetic frames and the real encoder but not with the Webots world.
