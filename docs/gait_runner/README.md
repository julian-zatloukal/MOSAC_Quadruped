# gait_runner: the latest Webots walk on the real robot

Firmware for the NUCLEO-F412ZG that runs the newest programmed gait: the forward walk of
`Webots/controllers/Programmed_Yaw`. That walk uses `walk_params.json`, laid over V2's `gait_params.json`,
with the stride ramp and stop of `motion_core.WalkController`. Every step goes through checks: the leg
starts where it is, every target is checked, and the servos are watched while they move.

The walk is a C port of `Programmed_Gait_V2/gait_core.py` and of the walking part of `Programmed_Yaw/motion_core.py`.
`make test` runs the C walk and the Python walk side by side and compares every step. They agree to within 5e-5°.

**Left out**, because the robot cannot use them yet:
- **The heading loop.** In Webots it reads the simulator's yaw. The IMU sketch computes a yaw but does not send it.
- **Turning, walking backwards and turning on the spot.** Without them, the four body (shoulder) joints stay at q = 0, as in
  `gait_core` when no turn is commanded.

Nothing here has run on the robot yet. See [What has been verified](#what-has-been-verified).

## Build (Docker)

```sh
cd docs/gait_runner
./build.sh                       # firmware -> build/gait_runner.bin / .hex / .elf   (RUN_MONITOR: no pulses)
./build.sh RUN_MODE=RUN_STAND    # any scalar setting of Src/config.h: RUN_MODE SPEED_SCALE STRIDE WALK_CYCLES POWERUP_WIGGLE
./build.sh test                  # C walk vs the Python walk, and gait_params.h vs the JSON files
./build.sh report                # the board's dry run of the walk, computed on the host
./build.sh params                # regenerate Src/gait_params.h after changing walk_params.json / gait_params.json
./build.sh clean
```

The first run builds the image `mosac-gait-runner-build:1`, which takes about 2 minutes. The image is Debian bookworm with
`gcc-arm-none-eabi` 12.2, plus gcc and python3-numpy for the tests. The whole repository is mounted, because:
- the HAL and the pot tables (`calibration_ADC.h`) come from `quadruped_control_mcu/`;
- the walk parameters come from `Webots/controllers/`.

**Flashing** is done on the host, not in Docker. Either:
- copy `build/gait_runner.bin` to the NUCLEO's USB drive (called `NOD_F412ZG`), or
- run `make flash` (OpenOCD) or use STM32CubeProgrammer.

**Serial:** ST-LINK virtual COM port, 115200 8N1. The log tool of leg_lab can record it:
`python3 ../leg_lab/tools/leg_lab_log.py record --port <port> --out walk.log`
(it needs pyserial).

## Before the first powered run

1. **Front-left femur (FFL).** This servo was swapped, and its pot does not match its table. The config marks it
   `POT_UNTRUSTED`, and gait_runner refuses to start until it is fixed:
   1. With its horn off, run leg_lab `PROGRAM_POTCAL` on it.
   2. Set `POT_LINE` and its two numbers in `CFG_POT_MODEL` / `CFG_POT_LINE`. These are the same names and format as in
      leg_lab's config.
2. **Calibrator pose of all 12 joints.**
   1. Fit the printed calibrators.
   2. Flash the default build (`RUN_MONITOR`) and press `c`.
   3. Paste the printed `CFG_CAL_POSE` line into `Src/config.h`.

   The defaults are the readings behind the old `quadruped_control_mcu` mid points. FFL is UNSET, because its value was
   the old servo's.
3. **Optional, recommended:** run leg_lab `PROGRAM_LIMITS` on each leg. Copy the `cmd` values of its `RES,limit` lines
   into `CFG_CMD_LIMITS`. No command then comes within 5° of a measured contact.
4. **Power.** Give the servos their own supply, and keep the Nucleo on USB. On 2026-11-24 the Nucleo browned out
   while servos stalled, and the pots shifted together (a ground shift). If VDDA falls below 3.0 V, every pulse is cut.
5. **Check the result.** Run `./build.sh report` and read the joint ranges in servo degrees. Each one must be a range
   the joint can really reach.

## Bring-up, one step at a time

| Step | Build | Robot | What to look at |
|---|---|---|---|
| 1 | `RUN_MONITOR` (default) | any | Pots and q stream once a second. Move each joint by hand: its q must change with the right sign |
| 2 | `RUN_STAND` | lifted, legs free | Power-up, crouch, stand up, stand check, sit down. Offsets and errors are printed |
| 3 | `RUN_STAND` | on the floor, lying | The same, now carrying its weight. The stand check must pass (errors ≤ 5°) |
| 4 | `RUN_WALK`, `SPEED_SCALE=0.3` | lifted | A walk in the air: tracking errors in the `fb` lines, no faults |
| 5 | `RUN_WALK`, `SPEED_SCALE=0.3`..`0.5` | on the floor | The real walk. Go faster with `+` only after clean runs |

## A session

USER, or `s`, starts a session in idle:

1. **Power-up.** Each servo switches on where it is, one at a time: the front-right, front-left, back-right, then back-left
   leg, each in the order body, femur, tibia. For each servo:
   1. The first pulse is sent where its pot says it is.
   2. Its leg's pots are watched every 1 ms for 0.3 s.
   3. It settles, and the pot offset is measured.
   4. A ±2° wiggle checks that the pot follows the servo.

   All 12 take about 30 s. Any failure cuts everything, so the robot must be lifted, or lying with its legs free.
2. **Crouch.** Eased joint move, peak 10°/s, to the feet under the hips at 110 mm.
3. **Stand up.** The hip-to-foot height rises from 110 to 149.8 mm with the feet straight under the hips. This is the
   `gait_core` standing pose.
4. **Stand check.** After 1.5 s, every joint must be within 5° of its command. If not, walking is disabled.
5. **Standing.**
   - USER or `w` walks (`RUN_WALK`).
   - `d` sits down.
   - `+` / `-` change the speed scale.
   - `p` prints the configuration and the plan.
6. **Walk.** First, a dry run from the current pose. Then the stride ramps up over 2 cycles, the robot walks
   `WALK_CYCLES` cycles, and it stops: the legs finish their stage and every foot lands centred. It then stands again.
   At the default speed 0.5 this takes 24.7 s.
7. **Sit down.** Back to 110 mm, then crouched:
   - USER switches off; the robot settles onto its belly.
   - `u` stands up again.

| Key | Idle | Standing | Walking | Holding (after a fault or a freeze) |
|---|---|---|---|---|
| USER | start | walk (`RUN_STAND`: sit down) | stop: finish the stage, centre the feet | ease back to the standing pose |
| USER again | | | freeze where it is | |
| `w` / `x` / `g` | | walk | stop / freeze | ease back to the standing pose |
| `d` | | sit down | | sit down |
| `+` `-` | speed | speed | | |
| `k` | | cut every pulse: the robot drops | same | same |
| `c` `a` `p` `h` | capture calibrators, raw ADC, config + plan, help | | | |

After a fault the firmware never moves the robot on its own: it holds and waits for you.

## Safety layers

| Layer | What it does |
|---|---|
| Default build | `RUN_MONITOR`: the servo pins are driven low and never reach the timers |
| Configuration check | Refuses any session if a pot is untrusted or a calibrator reading is missing, or if the crouch or standing pose leaves the windows |
| Dry run | Before every walk, the whole walk is computed from the current pose: ramp, cycles, stop and centring. Every target must be reachable and inside the windows. The speed scale is lowered until no joint needs more than 250°/s over 50 ms |
| Windows | Every target q must be inside `Q_WINDOW` (body ±5°, femur −30..40°, tibia −50..30°) and the tibia linkage window (q −63..69°). Its servo degrees must be inside the pulse range (5..261°) and at least 5° from any measured limit. A target outside is a fault, and it is never sent |
| Switch-on | The leg_lab probe. A pot of the leg that moves 3° for 5 ms, or 6° for 2 ms, cuts the pulse at once. Movement that the other legs' pots also show is electrical, and is ignored. Before the first pulse, every joint must be near its window and the tibia away from its dead points |
| Slew limits | Every command change goes through a hard limit: 20°/s outside the walk, 400°/s while walking |
| Tracking | The error between pot and command is averaged (100 ms). Above 6° while moving or standing, or 15° while walking, it is a fault |
| Fault | Every command freezes at its current value. A joint that is close to its command is set to its measured position. After that, a joint still off its command (blocked, or overloaded) gives way towards where it is, at 30°/s and at most 25°. If it is still more than 8° off after 1 s, everything is cut |
| Electrical watch | If the median error of all powered servos is over 4° (all pots shifting together), or VDDA dips by 0.05 V, everything pauses: the commands hold and the gait clock stops. A pause longer than 1 s is a fault. VDDA under 3.0 V cuts every pulse |
| Firmware fault | HardFault and the like: interrupts off, red LED on. The timers keep the last pulses, so the robot holds instead of dropping |

## Serial output

While powered, every 200 ms:

```
fb <s> <state> g <gait t> v <VDDA> cm <median error> BFR <sent deg>/<pot - sent> ... [PAUSED|FAULT]
```

State letters:
- `P` power-up
- `M` move
- `R` rise
- `S` stand
- `W` walk
- `w` walk stopping
- `H` holding
- `D` sit down
- `C` crouched

Event lines start with `FAULT:`, `CUT:` or `stand check`. In idle, the stream is `pot <s> BFR <pot deg> (q <q>) ... vdda`.

## Settings worth knowing (Src/config.h)

| Setting | Default | |
|---|---|---|
| `RUN_MODE` | `RUN_MONITOR` | `RUN_STAND`, `RUN_WALK` |
| `SPEED_SCALE` | 0.5 | Gait clock relative to Webots (1.0 = period 1.76 s). Lowered automatically for the joint speed limit; speed 1.0 becomes about 0.64 |
| `STRIDE` | 1.0 | Fraction of the 70.9 mm stride |
| `WALK_CYCLES` | 4 | Cycles at full stride per walk, after the 2-cycle ramp (1..12) |
| `MAX_JOINT_SPEED_DEG_S` | 250 | A DS3235 does about 400–460°/s with no load; nothing was measured under load |
| `H_LOW_MM` | 110 | Crouch height |
| `STAND_ERR_DEG`, `TRACK_*` | 5, 6 / 15 | Raise them only after reading real `fb` lines |
| `CFG_TRIM` | 0 | Per-servo mid point correction, e.g. from an inclinometer on the standing robot |
| `REF_Q` | 0, −42.65, −18.35 | q at the calibrator. The tibia value is open: `gait_core` uses 16.75 instead of 18.35 |

## What has been verified

- **C walk = Python walk.** `./build.sh test` runs four scenarios:
  - full speed;
  - half speed;
  - 0.3 speed at 60 % stride;
  - stop during the ramp, restart, stop during a swing.

  Stages, idle flags and all 8 joint targets match at every step, within 5.2e-5°.
- **Firmware build.** Builds with arm-none-eabi-gcc 12.2 with no warnings: 46 KB of flash and 25 KB of RAM in the RUN_WALK build.
- **Whole session, simulated.** The real `main.c` was run against a simulated board (in the session scratchpad, not in
  the repo). The simulated board had 12 servos with lag, pot offsets, current-dependent ground shift and VDDA sag.
  - **Normal run:** power-up, stand, walk, sit down, off. Switch-on probes moved a shaft at most 1.4°, and no joint moved
    more than about 1° in any 100 ms before the walk.
  - **Stops and faults:** graceful stop, freeze and recovery, femur blocked mid-walk (fault, the joint gives way 21°,
    holds without a cut, recovers), short ground shift (pause and resume), long ground shift (fault, recover), VDDA sag
    (cut), servo 20° off its pot at switch-on (cut after 5.4° of motion), `k` while walking, and the default configuration
    (refused).

**Not verified:**
- **Anything on hardware.** Every threshold comes from leg_lab and the 2026-11-24 log, not from walking.
- **How the DS3235 lags and sags under the robot's weight.** That decides whether the 6° and 15° tracking limits and the
  5° stand check are right.
- **Whether a slower gait clock walks the same on the floor.** The gait is statically stable, but Webots measured only
  speed 1.0 (44.6 mm/s).
- **The mid point convention.** The mid point is calibrator reading − dir·REF_Q + trim, which ignores the 0.1–2.9° pot
  offsets: a joint may be off by that much. Check the standing pose by eye or with an inclinometer, and use `CFG_TRIM`.

## Files

| File | |
|---|---|
| `Src/config.h` | Every setting |
| `Src/gait.c` | The walk: `gait_core` GaitGenerator + servo_angles + WalkController (forward) |
| `Src/gait_params.h` | Generated from the JSON files (`make params`) |
| `Src/plan.c` | The walk as the board runs it, and its dry run |
| `Src/pose.c` | q ↔ servo degrees, windows, IK poses |
| `Src/servo.c` | 10 ms tick, safety watch, switch-on, moves, freeze, fault hold, cut |
| `Src/pots.c` | Pot medians, table / line conversion, VDDA |
| `Src/main.c` | Session: power-up, crouch, stand up, stand check, walk, sit down; idle commands |
| `Src/hw_stm32.c`, `robot.c` | Hardware layer and pin map (from leg_lab, all 12 servos connected) |
| `tools/test_gait.py`, `gait_host.c` | `make test` |
| `tools/plan_host.c` | `make report` |
| `tools/gen_params.py` | `make params` |
| `docker/Dockerfile`, `build.sh` | The build environment |
