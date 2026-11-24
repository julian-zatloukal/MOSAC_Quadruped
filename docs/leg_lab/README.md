# leg_lab

Single-leg test firmware for the NUCLEO-F412ZG of the MOSAC quadruped. It replaces `docs/servo_test_leg` and keeps that firmware's hardware setup, which was run on the robot. The motion logic is rewritten around what those tests showed. You choose one leg and one program in `Src/config.h`. Only that leg's PWM pins are ever connected.

| Program | What it does | Moves the leg? |
|---|---|---|
| `PROGRAM_MONITOR` | Streams the pots. `r` records each joint's range while you move it by hand. `c` captures the pose with the printed calibrator fitted. | Never: no PWM pin is connected |
| `PROGRAM_CHECK` | Tests one servo at a time, the other two limp. It switches the servo on where it is, then sweeps ±5° and ±10° at 5°/s. It measures pot offset, gain, backlash and tracking. | Up to 10° around where the joint already is |
| `PROGRAM_POTCAL` | One servo with its horn off. It sweeps the full pulse range up and down and prints a new pot table in the `calibration_ADC.h` format. | The bare shaft only |
| `PROGRAM_LIMITS` | Robot lifted. A guarded search finds each joint's limits, then the femur × tibia and femur × body maps. | Yes, slowly, inside the windows described below |

## Order of work for one leg

1. **Monitor.** Flash `PROGRAM_MONITOR`, with `TEST_LEG` set to the leg.
   - Check that the three pots read steady, in-range values: no `INVALID`, and the spread (the number after the `/`) in single digits.
   - Move each joint slowly by hand, end to end. This works because the servos have no signal. Send `r`, then paste the printed `CFG_HAND_RANGE` into `config.h`.
   - Fit the printed calibrator and send `c`. Paste the printed `CFG_REF_POSE`.
2. **Check.** Flash `PROGRAM_CHECK` and press USER. Every servo should end with `PASS`. If one is refused or fails at switch-on, read the message: it names the servo and the cause.
3. **Limits.** Lift the robot so that this foot touches nothing. Flash `PROGRAM_LIMITS` and press USER. A run takes 10–20 minutes. Save the serial output to a file, or use `tools/leg_lab_log.py record`.
4. **Summary.** Run `python tools/leg_lab_log.py summary fr.log --gait --plot fr.png --json fr.json`. It gives:
   - the limits in command degrees, µs and Webots q;
   - the maps, and the foot positions they allow;
   - whether the `Programmed_Gait_V2` gait fits inside the measured ranges.

**The swapped front-left femur** has `POT_UNTRUSTED` in `CFG_POT_MODEL`, so every program except POTCAL refuses to switch it on. Calibrate it first:
1. Take its horn off the shaft.
2. Set `TEST_LEG LEG_FL`, `PROGRAM_POTCAL`, `POTCAL_JOINT JOINT_FEMUR` and `POTCAL_HORN_REMOVED 1`.
3. Run it. The result says whether the reading is a clean line. It prints:
   - a replacement for `ADC_VALUES_SERVO_6`;
   - an equivalent `POT_LINE` entry.

   Use one of them, and set the model to `POT_TABLE` or `POT_LINE`.
4. With `POTCAL_MOUNT_DEG` set, the program holds that angle at the end, so you can push the femur on at the nearest tooth.
5. Then redo step 1 for that leg, because the hand range and the calibrator pose are in pot degrees.

## Build and flash

Run this from this folder, on the machine that has `arm-none-eabi-gcc` and OpenOCD:

```
make                                             # settings from Src/config.h
make PROGRAM=PROGRAM_CHECK TEST_LEG=LEG_BR       # override without editing (rebuilds everything)
make flash                                       # the board then idles: nothing moves until USER or 's'
```

HAL, CMSIS and `calibration_ADC.h` come from `../../quadruped_control_mcu`. The serial port is the ST-LINK virtual COM port at 115200 8N1.

**Keys while idle:**
- `s` / USER: start
- `r`: hand range
- `c`: calibrator pose
- `a`: raw ADC of all 12 channels
- `p`: configuration and windows
- `h`: help

**Keys while running:**
- USER or `x`: ease back to the rest pose, then stop
- the same again, or `k`: cut all pulses at once

## Safety: what was learned and what the code does about it

| Seen on the robot (2026-11-17) | What leg_lab does |
|---|---|
| A servo sent to a fixed neutral at full speed slammed into its stop, stalled, and the board browned out | **No jump ever.** Each servo switches on where its pot says it is, one at a time. All later moves are eased at 8°/s peak, and the output stage enforces a hard limit of 12°/s on every command. No program uses a neutral angle |
| The front-left femur's pot and PWM disagree: a pulse at its own reading made it jump | **First-pulse probe.** All pots of the leg are watched every 1 ms. A pot more than 3° off for 5 ms, or 6° off for 2 ms, cuts the pulse. Readings that change faster than a servo can turn are treated as electrical. Untrusted pots are never switched on |
| Brief pot dips when a motor starts caused false trips | Dips also seen on the reference pots, or faster than a servo can move, are ignored |
| The back-right tibia's pot read a fixed 250° (no supply) | **Wiggle.** After switch-on, a slow ±2° move: the pot must follow (gain ≥ 0.5). An ADC spread of 0 is also shown |
| All pots shifted together by up to 24° during the stall, before the brownout | **Electrical watch.** The 9 pots of the unpowered legs cannot move, so they serve as references, along with VDDA (from VREFINT): a shift of 2° pauses, 6° cuts at once, and a pause longer than 1 s also cuts |
| | **Ground-shift correction.** Their median shift is subtracted from the leg's pots |
| | **Stall sign.** During a search, a reference shift above 1° is taken as stall current, and the search stops |
| A cut makes a lifted leg fall | **Freeze before cut.** Contacts and blocks command the joint where it actually is, so it stops pushing and keeps holding. Pulses are cut only in emergencies, or once the joint is back at its rest pose: the pose the limp leg was in at switch-on, so nothing drops |
| | **Stops:** USER eases back to the rest pose before cutting; a second press, or `k`, cuts at once |
| Neutral angles in the repo are unverified, and some are outside the leg's range | **Windows.** Every joint has one: the pulse range, a travel cap around the start, the hand range + 3°, and, for the tibia, the linkage model. The output stage never sends a command outside it |

## How the limit search treats the leg's structure

The thing that stops a joint is not always a hard stop. The program handles four cases.

1. **Hard stops and collisions**, such as the femur against the body, the tibia against the femur, or a link against a link. A servo cannot report force, so contact is found by **lag**: the joint moves at a steady 4°/s and the program watches how far its pot falls behind the command.
   - **Baseline:** the lag it has while moving freely is learned first, so gravity load does not count.
   - **Onset:** a rise of 1° above that is taken as the start of contact, and from then on the command advances at only 1°/s.
   - **Contact:** 2° above it for 100 ms ends the search. The joint is commanded where it is, backs off 4°, and approaches again at 1.5°/s. That gives a repeatability figure. The limit reported is where the contact began, not where the push ended.
   - **Contact through another link** shows on the held joints. If one is pushed 2.5° for 100 ms, that is also contact.
2. **The tibia four-bar linkage.** The tibia servo drives the knee through a linkage with A 24, B 40.5, C 28.66 and D 27 mm. It is not a Grashof linkage, so it has toggle positions. From the model in `gait_core.py`:
   - **Toward higher q:** the coupler and rocker line up at about q = +89°. The crank binds there, and every joint of the linkage takes the full servo torque.
   - **Toward lower q:** the coupler folds onto the crank at about q = −130°. The knee stops and reverses while the servo keeps turning. Nothing blocks, so lag detection cannot see it.

   Both transmission angles stay at 30° or more for q in −63..+69°, which is `TIBIA_Q_SAFE`. With the calibrator pose set, that range is part of the tibia's window. A joint that suddenly runs ahead of its command (snap-through) also stops the search. Because contact detection alone cannot guard the linkage, **the tibia is only searched when it has a hand range or a calibrator pose**.
3. **Coupled limits.** Whether the tibia can fold depends on where the femur is, and so does the body's swing. After the single-joint limits, the femur is stepped across its range in 15° steps, keeping 5° inside its limits. At each step the tibia limits are searched. The body limits are searched with the femur near each end. These are the boundaries a kinematics model needs: the reachable set is a region in (femur, tibia), not a box.
4. **Limits from the electrical side.** The stall sign described above can also end a search. Such a result is marked `electrical`, and the search is retried once, because a disturbance from outside does not repeat at the same angle.

Every result is printed both as text and as a `RES,...` line. `tools/leg_lab_log.py` uses the calibrator pose and the `gait_core` geometry to turn them into:
- Webots joint angles q;
- foot positions, as hip-to-foot height H and foot offset D1;
- a check against the gait's trajectory.

`python tools/leg_lab_log.py selftest` checks its forward kinematics against `gait_core.servo_angles`.

## Not verified on the robot

This firmware has only been compiled for the Cortex-M4 with clang, and run against a simulated leg. It has not been built with arm-none-eabi-gcc, linked, or flashed. The simulation had stops, pot offsets, a pot/PWM mismatch, stall current and ground shift, and it caught two bugs that were fixed:
- the USER press that starts a program was taken as a stop;
- ground shift hid contacts on one side.

These thresholds are guesses until the real leg is measured. Watch the telemetry (`ref` and `cm`) on the first runs and adjust them in `config.h`:
- `SEARCH_REF_DEG`
- `REF_PAUSE_DEG`
- `FB_COMMON_MODE`
- the contact thresholds

Other open points:
- **Cut latency:** when pulses stop, the DS3235 is expected to stop within a frame. The log suggests it goes limp, but how far a servo with a mismatched pot moves before the cut takes effect is unmeasured. In the worst-case simulation it was 7°.
- **Linkage figures:** the toggle angles come from the CAD dimensions in `gait_core.py`, and from the calibrator q of `main.h`. For the tibia, `gait_core` uses 16.75° where `main.h` uses 18.35°.
- **Current sensing:** a sensor on the servo supply, such as an INA219, would give a direct stall signal. It is the most useful hardware addition for these tests.
