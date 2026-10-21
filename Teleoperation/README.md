# Teleoperation

A web page that shows the robot's camera full screen and walks the robot in Webots, forward and backwards, and turns it on
the spot. Made for a phone first (portrait or landscape), and it works the same on a desktop. The page is hosted by one Python
script (standard library only); the camera needs ffmpeg on the robot's side (`brew install ffmpeg`).

```
1. Open Webots/worlds/MOSAC_REAL_V4_teleop.wbt in Webots (the robot stands up and waits). ./open_webots.sh does it in a new window.
2. python3 Teleoperation/server.py        opens http://127.0.0.1:8000 in the browser
```

| control | does |
|---|---|
| **▲** (hold, left thumb) | walk forward, holding the heading. Keys: `↑` `W` |
| **▼** (hold, left thumb) | walk backwards, holding the heading. Keys: `↓` `S` |
| **⟲** / **⟳** (hold, right thumb) | yaw left / right: turn on the spot, or walk a gentle curve if ▲ or ▼ is held too. Keys: `←` `A` / `→` `D` |
| **■** (`Space`, `Esc`) | stop |
| **Speed** (under ■) | 10, 25, 50, 75 or 100%. Scales the stride (44.8 mm/s forward and 50.9 mm/s backwards at 100%) and the turn (7.8 deg/s on the spot, 2.3 deg/s on a curve) |
| top right | fill the screen with the video (cropped) or show the whole picture; full screen, where the browser allows it (not on iPhone) |

Several can be held at once, like WASD; ▲ and ▼ together cancel out. The yaw buttons turn the robot's nose that way whichever
way it walks, so ▼ with ⟲ swings the nose to the left (counter-clockwise) and the tail to the right, unlike a car's steering
wheel when reversing. Walking forward, walking backwards and turning on the spot use three different gaits, so going from one
to another makes the robot stop and re-centre its legs first: 2 to 2.7 s.

The pill at the top left shows whether the robot is connected (green), the heading, and what it is doing (Walking, Reversing,
Turning, Stopping while the legs re-centre, Standing). The key letters on the buttons only show with a mouse.

## The camera

A cheap USB webcam on a stand at the front of the body's top plate (`camera` in `MOSAC_REAL_V4_teleop.wbt`): 1280x720, 63 deg
horizontal field of view, 22 cm above the ground and tilted 10 deg down, so the ground shows from about 0.4 m in front of
it. It has no mass and no collision
shape, so the tuned gaits are not affected. The robot side (`Webots/controllers/Programmed_Yaw/camera_stream.py`) renders it
at 30 fps and encodes H.264 with ffmpeg (libx264 `veryfast`, `zerolatency`, a key frame every second). Measured on the M4 with
the view moving like the walking robot's: 7 ms to encode a frame, 2.7 Mbit/s, 60% of one core, and the camera costs Webots
about 2.5 ms per frame. Nothing is rendered or encoded while no page shows the video, and the camera switches off 3 s after
the last page leaves.

Browsers show it in one of two ways (`video.js`), with this delay from the camera to the screen measured in Chrome on the same
computer:

- **WebCodecs**, on `localhost` or https pages: each frame is decoded as it arrives and drawn on a canvas. 8 ms.
- **Media Source Extensions**, which work on plain http too, so this is what a phone on the network gets: the frames are packed
  into MP4 fragments and played by a `<video>` kept close to the newest frame. 65 ms. iPhones need iOS 17.1 or later.

A new page gets a picture within a second (it waits for the next key frame). `?video=mse` or `?video=webcodecs` in the address
forces one of the two.

## How it works

```
browser --HTTP POST /api/command, every 100 ms while held--> server.py --UDP 9871--> Webots: Programmed_Yaw --teleop
browser <--GET /api/state, 5 times a second------------------ server.py <--UDP 9872-- (heading, position, state, every 100 ms)
browser <--WebSocket /api/video, one message per frame------- server.py <--TCP 9873-- camera_stream.py (H.264, while watched)
```

The command is `{"cmd": "drive", "forward": -1..1, "turn": -1..1}` (negative forward = backwards) or `{"cmd": "stop"}`
(`protocol.py`, shared by both sides, also describes the video messages). On the Webots side `Programmed_Yaw.py --teleop`
hands the commands to a `MotionController` (`Webots/controllers/Programmed_Yaw/remote.py` and `motion_core.py`), so stopping,
reversing, the speed ramps and walking behave as described in that README. The server only passes the video on: it connects to
the camera while some page shows it, and a page that falls behind (slow Wi-Fi) skips to the next key frame instead of building
up delay.

## Safety

- **The robot stops by itself.** A drive command lasts 0.5 s, so the page has to keep repeating it. If the page is closed, the
  network drops, or the server is killed, the robot stops within 0.5 s (measured: 0.55 s). The server also
  sends a stop when it is shut down with Ctrl+C.
- **The page lets go on its own** when the button, key or touch is released, the window loses focus, or the tab is hidden.
- **Only this computer can reach it** (`127.0.0.1`). There is no login, so `--host 0.0.0.0` lets anything on the network drive
  the robot and watch its camera; do not use it on a network you do not trust. Requests must be JSON, from the page's own
  origin, to the server's own host name, and the video WebSocket must come from the page's own origin too, which keeps other
  websites open in your browser from steering the robot or reading the camera.

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

- `server.py` the web server, the UDP link and the video relay. `index.html` the page. `video.js` the video player.
  `protocol.py` the messages.
- `python3 test_teleop.py` runs the page's server against the real robot-side code over loopback, without Webots: commands,
  the timeout, reversals, and the requests the server must refuse. It does not cover the video.

## Troubleshooting

- *No robot* on the page: the world is not open, the simulation is paused, or the UDP ports are taken (Webots' console says so).
  Only one `MOSAC_REAL_V4_teleop.wbt` can run at a time.
- *No video* with the robot connected: Webots' console says why after `[video]`, usually that ffmpeg was not found.
- *Video not supported*: the browser can play neither way (an iPhone before iOS 17.1, for example).
- *Cannot start ... try --port*: something else is on port 8000.

## Not done yet

No walking sideways, and `TurnController.go_to(heading)` exists but is not in the protocol or the page. The
receiver is the Webots controller; the real robot has no counterpart yet (neither for the commands nor for a camera).
Checked in Chrome (desktop, and phone screen sizes) only, not in Safari or Firefox or on a real phone.
