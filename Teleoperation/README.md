# Teleoperation

A web page that walks the robot in Webots and turns it on the spot. The page is hosted by one Python script (standard library only,
nothing to install).

```
1. Open Webots/worlds/MOSAC_REAL_V4_teleop.wbt in Webots (the robot stands up and waits).
2. python3 Teleoperation/server.py        opens http://127.0.0.1:8000 in the browser
```

| control | does |
|---|---|
| **Forward** (hold) | walk forward, holding the heading. Keys: `↑` `W` |
| **Yaw left** / **Yaw right** (hold) | turn on the spot, or walk a gentle curve if Forward is held too. Keys: `←` `A` / `→` `D` |
| **Stop** (`Space`, `Esc`) | halt |
| **Speed** slider | 10 to 100%. Scales the stride (44.6 mm/s at 100%) and the turn (7.8 deg/s on the spot, 2.3 deg/s on a curve) |

Several can be held at once, like WASD. Walking and turning on the spot use different gaits, so going from one to the other
makes the robot stop and re-centre its legs first: about 2 s from walking to turning, 3 s the other way.

The page also shows the robot's heading and position and what it is doing (walking, turning, stopping while the legs
re-centre, standing), and whether the robot is connected.

## How it works

```
browser --HTTP POST /api/command, every 100 ms while held--> server.py --UDP 9871--> Webots: Programmed_Yaw --teleop
browser <--GET /api/state, 5 times a second------------------ server.py <--UDP 9872-- (heading, position, state, every 100 ms)
```

The command is `{"cmd": "drive", "forward": 0..1, "turn": -1..1}` or `{"cmd": "stop"}` (`protocol.py`, shared by both sides). On
the Webots side `Programmed_Yaw.py --teleop` hands them to a `MotionController` (`Webots/controllers/Programmed_Yaw/remote.py`
and `motion_core.py`), so stopping, reversing, the speed ramps and walking behave as described in that README.

## Safety

- **The robot stops by itself.** A drive command lasts 0.5 s, so the page has to keep repeating it. If the page is closed, the
  network drops, or the server is killed, the robot stops within 0.5 s (measured: 0.55 s). The server also
  sends a stop when it is shut down with Ctrl+C.
- **The page lets go on its own** when the button, key or touch is released, the window loses focus, or the tab is hidden.
- **Only this computer can reach it** (`127.0.0.1`). There is no login, so `--host 0.0.0.0` lets anything on the network drive
  the robot; do not use it on a network you do not trust. Requests must be JSON, from the page's own origin, to the server's
  own host name, which keeps other websites open in your browser from steering the robot.

## Options

```
python3 server.py --port 8001 --no-browser
python3 server.py --robot-host 192.168.1.50 --status-bind 0.0.0.0     the robot's controller is on another computer
```

On the robot's side `TELEOP_BIND` (where it listens, default `127.0.0.1`) and `TELEOP_SERVER_HOST` (where it sends the status)
are read from the environment of Webots. The UDP ports are at the top of `protocol.py`.

## Files and tests

- `server.py` the web server and the UDP link. `index.html` the page. `protocol.py` the messages.
- `python3 test_teleop.py` runs the page's server against the real robot-side code over loopback, without Webots: commands,
  the timeout, reversals, and the requests the server must refuse.

## Troubleshooting

- *No robot* in the corner of the page: the world is not open, the simulation is paused, or the UDP ports are taken (Webots'
  console says so). Only one `MOSAC_REAL_V4_teleop.wbt` can run at a time.
- *Cannot start ... try --port*: something else is on port 8000.

## Not done yet

No walking backwards or sideways, and `TurnController.go_to(heading)` exists but is not in the protocol or the page. The
receiver is the Webots controller; the real robot has no counterpart yet.
