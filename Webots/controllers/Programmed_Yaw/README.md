# Programmed_Yaw: turning on the spot, and walking

The crawl gait of [Programmed_Gait_V2](../Programmed_Gait_V2) turning in place instead of walking. Three commands:

```python
turner.turn(rate)      # hold a turn; rate is a fraction of the full speed in [-1, 1], positive = left. turn(0) stops.
turner.go_to(heading)  # turn the short way to a heading in rad (world frame) and stop there
turner.stop()          # halt
```

`Programmed_Yaw.py` plays a scripted session with them (hold a turn, stop, go to +90 deg, go to -45 deg, progress printed to
the Webots console). With `--teleop` the commands come from the web page in [Teleoperation](../../../Teleoperation) instead
(`remote.py`: it listens on UDP, stops the robot when the commands stop, reports the heading back), and the robot can also
walk forward and backwards ([Walking](#walking) below). The webcam on the robot in that world is streamed to the page while the
page shows it (`camera_stream.py`: 30 fps, H.264 through ffmpeg, TCP; see the Teleoperation README). Any other input would replace `Session` or `RemoteControl` the same way: read a
keyboard or joystick and call the commands every control step.

```
open Webots/worlds/MOSAC_REAL_V4_yaw.wbt           run the session
open Webots/worlds/MOSAC_REAL_V4_teleop.wbt        wait for the web page (python3 Teleoperation/server.py)
python test_turn.py, python test_motion.py          offline tests, no Webots
python check_turn.py                                what the turn speed settings give, no Webots
```

## How it behaves

- **Stopping is immediate.** `stop()` zeroes the stride, so the rotation halts in the same control step (in simulation at most
  ~1 deg of extra rotation from the servos catching up). The leg that is in the air finishes its swing and lands at the centre,
  and after one gait cycle every foot is centred. Then `turner.idle` is True and the controller stops stepping the gait: the
  robot holds a symmetric standing pose until the next command.
- **Starting takes up to three stages** (0.4 s at the default period): the body only moves in the push stages, once every
  quarter cycle.
- **Speeding up is ramped** (`ramp_cycles`, one gait cycle from rest to full speed). Slowing down and stopping are not.
- **Reversing waits.** Asking for the other direction while the feet are still set up for the first stops the rotation at once,
  lets the feet centre (up to one gait cycle) and then starts the new direction.
- **`go_to`** turns at `kp * error` (limited to full speed, at least `omega_min`) and stops when the error is below `tol_deg`.
  In simulation it ends within ~1.4 deg of the target for turns from 5 to 179 deg.

Why the ramp and the reversal wait: a swing travels `|total push| + (|omega next| - |omega before|) * lever / 2`. Speeding up
or reversing faster than the feet can follow makes some foot swing up to twice as far as in a steady turn, and the joints up
to twice as fast (509 deg/s was measured offline for a reversal at the wrong moment, against 246 deg/s in a steady turn).
With the ramp and the wait, 1500 random commands never moved a joint faster than a full speed turn from rest (`test_turn.py`).

## Changing the speed

```
yaw rate at full command (deg/s) = degrees(omega_max) / gait_period
```

Edit `turn_params.json`; the three that matter:

| key | meaning | to turn faster | what limits it |
|---|---|---|---|
| `omega_max` | rad turned per gait cycle at full command | raise | shoulder angle, about 31 deg per rad |
| `gait_period` | seconds per gait cycle | lower | joint speed, about proportional to 1/period |
| `parabola_height` | how high a foot is lifted, mm | lower it, so the period can be shorter | foot clearance |

The fastest joint movement comes from lifting the foot, so a shorter lift lets the period shrink. The other keys are
`ramp_cycles` (start-up), `kp`, `omega_min` and `tol_deg` (`go_to`). Anything else in `Programmed_Gait_V2/gait_params.json`
(standing height, lean) is shared with the walking gait.

**Before trying new values, run `python check_turn.py`.** It runs the real controller offline and prints the yaw rate, the
fastest femur/tibia movement against the 350 deg/s servo limit assumed in `tune_gait.py`, and the largest shoulder angle,
with a warning when a limit is passed:

```
python check_turn.py                                     the values in turn_params.json
python check_turn.py --period 1.8 --lift 12 --omega 0.3  try others without editing the file
python check_turn.py --table                             peak joint speed for periods x foot lifts
```

`TURN_PARAMS=<file>` makes the Webots controller read another file (set it in the environment of the `webots` process).

Presets (foot lift 12 mm, `omega_max` 0.3 unless noted; `check_turn.py` numbers, then the simulated time of a 134 deg `go_to`):

| `gait_period` | yaw rate | peak joint speed | shoulder | 134 deg turn in simulation |
|---|---|---|---|---|
| 3.2 s | 5.4 deg/s | 226 deg/s | 9.1 deg | 26 s |
| **2.2 s (default)** | 7.8 deg/s | 270 deg/s | 9.2 deg | 19 s |
| 1.8 s | 9.5 deg/s | 329 deg/s | 9.4 deg | 16 s |
| 2.2 s, `omega_max` 0.2 | 5.2 deg/s | 209 deg/s | 6.1 deg | |
| 2.2 s, `omega_max` 0.35 | 9.1 deg/s | 303 deg/s | 10.7 deg (warning) | |
| 2.2 s, `omega_max` 0.4 | 10.4 deg/s | 339 deg/s | 12.1 deg (warning) | |

In simulation the robot turned within 7% of `omega_max / gait_period`. A slower turn than the configured one needs no edit:
`turn(0.5)` is half speed.

## Walking

In teleop mode the controller is a `MotionController` (`motion_core.py`) with one command, `drive(forward, turn)`: `forward` is
a fraction of the full stride in [-1, 1] (negative = backwards) and `turn` a fraction of the full turn in [-1, 1]; `stop()` halts.

| command | does |
|---|---|
| `forward > 0`, `turn = 0` | walks straight. The heading when it started (or when the turn was let go) is held by the same PI loop as `Programmed_Gait_V2`, so a push or a drift is corrected |
| `forward > 0`, `turn != 0` | walks a gentle curve, 2.3 deg/s at full turn; letting the turn go holds the heading it ended on |
| `forward < 0` | walks backwards, the same way, with a gait of its own ([Walking backwards](#walking-backwards)). A positive `turn` still turns the nose to the left (counter-clockwise) |
| `forward = 0`, `turn != 0` | turns on the spot, as above |

Measured in simulation, in real time through the Teleoperation page: **44.6 mm/s** straight, 0.13 mm/s sideways drift, 0.2 deg of
heading change over 16 s; 22.1 mm/s at half the stride; holding a curve then letting go kept the heading within 1 deg. Stopping
and re-centring the legs takes 1.9 s, and going from walking to turning on the spot 1.9 s (2.7 s the other way).

### Changing the walking speed

The pace is the stride divided by the period, and what limits it is how fast the joints have to move. The joints move fast
only in the swing stage (a foot lifted and carried forward), yet in the original gait it gets a quarter of every cycle, the same
as the lean, restore and push stages. So walking is sped up by sharing the cycle out differently, with weights in
`walk_params.json`:

| key | meaning |
|---|---|
| `gait_period` | seconds per gait cycle (1.763) |
| `parabola_length` | stride in mm (70.9) |
| `parabola_height` | foot lift in mm (10.6) |
| `weight_lean`, `weight_swing`, `weight_restore`, `weight_push` | how the cycle is shared out between the four kinds of stage; only the ratios matter. 0.33 : 1 : 0.5 : 0.35 gives 67, 202, 101 and 71 ms where the original gait has 110 ms for each |
| `ramp_cycles` | gait cycles the stride takes to grow from nothing to full (2, so 3.5 s) |
| `turn_max` | largest turn while walking, rad per gait cycle (0.07), for the heading loop and for a commanded curve |

The standing height and the lean are shared with the turning gait (from `Programmed_Gait_V2/gait_params.json`), because the gait
has to switch between the two without moving the legs. Everything else here can be changed. To find a new pace, search for it
instead of guessing, with the tuner of `Programmed_Gait_V2` (which runs the gait in Webots and scores speed, drift, tilt and
torque, rejecting candidates that move the joints faster than 350 deg/s in a steady walk):

```
python -c "import json, motion_core; json.dump(motion_core.load_walk_params()[0], open('walk_base.json', 'w'))"
cd ../Programmed_Gait_V2
WEBOTS=<webots executable> python tune_gait.py --base ../Programmed_Yaw/walk_base.json --check-cycles 3 --workers 5 \
  --search gait_period parabola_length parabola_height weight_lean weight_restore weight_push \
  --range gait_period=1.5:4.5 parabola_length=70:130 parabola_height=8:16 weight_lean=0.3:2 weight_restore=0.3:2 weight_push=0.3:2
```

The search of 1488 candidates took 11 minutes and found several gaits at about 45 mm/s; this one has the lowest joint speeds of the
best four. Copy `best_params.json` into `walk_params.json` (keeping `weight_swing`, `ramp_cycles` and `turn_max`), then run
`test_motion.py`. `forward` scales the stride, so the slider of the page trades pace for joint speed linearly.

### Walking backwards

`forward < 0` runs a third gait, `back_params.json` (the same keys as `walk_params.json`). The inverse kinematics is the same
function (`servo_angles`: foot height and offset from the hip to femur and tibia angles; the leg does not care which way the foot
travels). What is different is the foot trajectory:

- **The swings come in the reverse order**, BL, FR, BR, FL instead of FL, BR, FR, BL (the `backward` gait parameter,
  `STAGES_BACKWARD` in `gait_core.py`), each still after the lean meant for it. The feet swing from front to rear and the pushes
  slide them forward. This is the forward gait played backwards in time: every pose is one the forward walk goes through (within
  0.5 mm, checked offline), so the body stands on the same support triangles. Walking backwards with the forward order also
  works, but tilted the body more in Webots (9.0 deg against 6.5 deg).
- **The parameters were searched** like the forward ones. The tuner scores the speed backwards for a gait with `backward` set:

```
python -c "import json, motion_core; json.dump(dict(motion_core.load_walk_params()[0], backward=True), open('back_base.json', 'w'))"
cd ../Programmed_Gait_V2
WEBOTS=<webots executable> python tune_gait.py --base ../Programmed_Yaw/back_base.json --check-cycles 3 --workers 6 \
  --init-size 64 --maxiter 20 --search gait_period parabola_length parabola_height weight_lean weight_restore weight_push \
  --range gait_period=1.5:4.5 parabola_length=70:130 parabola_height=8:16 weight_lean=0.3:2 weight_restore=0.3:2 weight_push=0.3:2
```

The search started from the forward walk (44.9 mm/s backwards with the reversed order) and simulated 1344 candidates in 13
minutes; none fell. The best six all did 51 to 54 mm/s, and moved the joints alike after random commands (392 to 399 deg/s).
`back_params.json` is the one with the lowest joint speeds, as for the forward walk: second fastest (53.2 mm/s against 53.7),
the gentlest on a curve, and on rough ground it tilted less than the fastest (4.2 deg against 6.5).

| | forward (`walk_params.json`) | backwards (`back_params.json`) |
|---|---|---|
| period, stride, foot lift | 1.763 s, 70.9 mm, 10.6 mm | 1.567 s, 74.2 mm, 9.9 mm |
| stage weights lean : swing : restore : push | 0.33 : 1 : 0.5 : 0.35 | 0.30 : 1 : 0.40 : 0.30 |
| tuner, flat floor | 45.0 mm/s, tilt 6.0 deg | 53.2 mm/s, tilt 6.6 deg |
| tuner, `MOSAC_REAL_V4_terreno_irregular` | 38.9 mm/s, tilt 4.8 deg | 40.7 mm/s, tilt 4.2 deg |
| real time through the Teleoperation page | 44.8 mm/s | 50.9 mm/s (25.2 at half the stride) |
| joint speed offline, steady walk / curve at full turn | 330 / 360 deg/s | 333 / 339 deg/s |

Through the page, in real time: walking backwards held the heading within 1 deg over 16 s, with a 1.4 mm/s sideways creep (the
tuner measured 0.2); a curve at full turn yawed 2.3 deg/s, and letting the turn go held the new heading within 0.5 deg. Stopping
took 1.7 to 1.9 s to centre the legs, with the body settling within 5 mm. Switching took 2.1 s backwards to forward, 2.3 s forward
to backwards, 2.0 s backwards to turning on the spot and 2.7 s back. The page's watchdog stopped it 0.63 s after the commands
stopped.

The lean and push weights ended at the bottom of the search range (0.3), as they did for the forward walk, so shorter stages may
be faster still; the standing height and the lean cannot be searched here, because all three gaits have to share them.

### Behaviour

- **Stopping** is immediate, as for turning: the stride is zeroed in the same step. The legs then re-centre in one gait cycle
  (1.8 s), and the body settles by about 5 mm; then `idle`.
- **Switching between walking forward, walking backwards and turning on the spot waits.** They use different gait parameters (a
  period of 1.76 s, 1.57 s and 2.2 s, the stages shared out differently, and another swing order backwards), which can only be
  changed with the feet centred. So the running gait stops, the legs re-centre, and the other one starts, carrying on from the
  same lean. `test_motion.py` checks that no joint target jumps at the switch.
- **Foot lift.** A stride that changes needs `finish_stages` on (a swing has to start from where the foot is), and with it on the
  old 21.7 mm lift moved the joints at 409 deg/s and was rejected by the tuner. The search found 10.6 mm to be enough.
- **Joint speed.** 328 deg/s in a steady walk (offline; 332 measured in the tuner's simulation, against 371 for the old walk).
  A curve at the full turn is 360 deg/s. Starting, stopping and changing the command quickly can reach about 400 deg/s (403, the
  worst of the random command runs in `test_motion.py`, forward, backwards and turning, 21% above the steady walk), above the
  350 deg/s assumed for the servos and below the ~490 deg/s no load speed of the motor model.

## Limits and what is not verified

- 350 deg/s is an assumption (the motor model in `MOSAC_Controller` has ~490 deg/s no load), and the simulator does not enforce it.
  Walking exceeds it briefly, see above.
- The shoulder warning is at 10 deg, the smaller side of the -10..+15 deg range the policy in `quadruped_control_rpi` gives the
  shoulders. The mechanical range of the real robot was not checked.
- The foot lifts of 10 to 12 mm were tried in the teleop controller on the flat floor only. The tuner walked both directions on
  `MOSAC_REAL_V4_terreno_irregular` without falling (table above), but turning on the spot there was not tried; if the feet
  catch, raise the lift and expect to lower the pace to stay under the joint speed limit.
- The heading is read from the simulator. On the real robot it would come from integrating the IMU gyro, which drifts; fine for
  turns of a few tens of degrees, and `turn()` needs no heading at all.
- `MOSAC_Controller` uses a 3 deg servo dead band, which may swallow the shoulder moves of a slow turn (below `omega_max` ~0.1).

## Changes to Programmed_Gait_V2

`gait_core.py` got six additions. None of them changes the walking gait: `test_equivalence.py` still matches the original to
7e-7 rad, and the tuned walk still does 26.1 mm/s in Webots.

- `set_twist(..., forward=None)`: 0 turns on the spot, negative walks backwards, None is the full stride in the gait's direction.
- The `backward` parameter (false by default): the stage table in the reverse order (`STAGES_BACKWARD`), the start pose staggered
  the other way, and `set_twist()` walking backwards. `use_params()` can switch direction while the feet are centred: it carries
  on from the stage of the other table that leans the same legs (`mirror_stage`). `tune_gait.py` scores the speed backwards for
  such a gait (`--set backward=true`).
- The landing point of a swing is fixed when the swing starts, so a command can change at any moment without a foot jumping.
- `use_params()`: carry on with other gait parameters (a different period and foot lift) from the same stage and progress, which is
  how a stopped gait changes between turning and walking.
- Stage weights (`weight_lean`, `weight_swing`, `weight_restore`, `weight_push` in `gait_config.py`, all 1 by default, which is the
  original gait): each kind of stage takes a share of the cycle in proportion to its weight. `use_params()` keeps the stage and
  the progress in it, so it works between sets with different weights. `tune_gait.py` can search them (`--search`, `--range`,
  `--set`, `--check-cycles`).
- `stand()`: feet centred, the pose a turn on the spot starts from. The walking start pose staggers the feet, and a right turn
  started from it moved the joints 20% faster than a left one (307 against 256 deg/s in simulation).

Turning needs `finish_stages` on, which `load_turn_params()` forces: with it off, a swing starts from the nominal rear end of
the stride instead of from where the foot is, and the joints reached 754 deg/s.
