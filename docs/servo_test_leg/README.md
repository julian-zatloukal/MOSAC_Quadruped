# servo_test_leg

A small test firmware for the NUCLEO-F412ZG. It moves the selected legs **slowly** through a fixed sequence of joint sweeps and reads each servo's pot feedback. It runs open loop: it doesn't use the Raspberry Pi or the SPI link. The feedback is used to start without jumps and to stop on faults, not to control the servos. Only the PWM pins of the selected legs are configured. Servos on any other leg get no signal.

## Programs

Set `PROGRAM` at the top of `Src/main.c`:

- **`PROGRAM_CHECK`** (the default) is a troubleshooting program for any leg, including ones whose neutral angles or calibration aren't verified. It never goes to neutral. It tests one servo at a time:
  1. With pulses off, it checks that the pot reading is in range and steady.
  2. It sends the first pulse at the servo's own measured angle and watches all three pots of the leg every 1 ms. If any pot moves more than 4°, that servo is cut at once. This catches a pot/PWM mismatch, or a PWM line that drives a different servo, after a few degrees of movement.
  3. It sweeps ±5° and then ±10° around that position at 5°/s, with the fault cut-out active. This measures the pot offset, the gain (pot change ÷ commanded change, which should be about 1, judged on the ±10° sweep), the tracking error, and how much the leg's other pots move.
  4. The servo goes limp, and a summary is printed at the end.

  A full leg takes about 65 s.
- **`PROGRAM_LIMITS`** finds each joint's mechanical limits.
  1. All pots of the leg must be valid.
  2. Every joint is switched on at its measured angle with the same 1 ms probe, and then holds there, so the leg's shape stays fixed.
  3. One joint at a time moves at a steady 5°/s. A limit is detected when its pot falls 3° behind the command for 150 ms, meaning the joint is blocked or can't follow. It stops pushing at once (the command is set to where the joint actually is), backs off 5°, returns to the start, and then searches the other direction.
  4. A search never goes more than 135° from the start, or outside 5–260°. The fault cut-out stays active on the other joints.
  5. It prints each joint's low and high limits in degrees and µs, the range, and whether neutral lies inside it. Choose the joints with `LIMIT_JOINTS`.
- **`PROGRAM_SEQUENCE`** runs the slow test sequence around neutral, described below. Use it only on legs that passed the check and whose neutral angles are known.
- `MOTION_ENABLED 0` builds a feedback-only version: no pulse can be sent. Use it to read the pots while you move joints by hand, with `r` printing the min..max range of each pot.

All 12 servo signal lines are driven low from the first instructions after reset, so no servo sees a floating line. While idle, the reason for the last stop is printed every 5 s.

### Check results, 2026-11-17

| Servo | Result |
|---|---|
| BFR, FFR, TFR | Pot offset about −2.4°, tracks well (tested with the sequence program) |
| BFL | Holds its first pulse, offset −0.1° |
| FFL | **Pot and PWM disagree by about 20–28°.** The first pulse moves the femur hard upwards. |
| BBR | PASS: offset −2.5°, gain 1.0 over ±10° (0.8 over ±5°, a little backlash) |
| FBR | PASS: offset −0.6°, gain 0.9 |
| TBR | PASS after a reboot: offset −2.9°, gain 0.9. Before the reboot its pot read a fixed 250°, and it was cut on its first pulse. The pot most likely had no power (connector or supply), so check that the TBR connector is seated. |

| BBL | Feedback OK and holds its first pulse (offset −1.8°), but it **falls short when moving to higher angles**: −1.5° at +5° and −3.6° at +10° (pot only reached 124.3° for 127.9°). The − direction is fine (+1.1°). Likely load or a mechanical limit in that direction. |
| FBL | PASS: offset −2.6°, gain 1.0, tracking 1.7° |
| TBL | PASS: offset −0.8°, gain 1.0, tracking 1.3° |

The back-right joints sit far from their `Programmed_Gait` neutrals: FBR is at about 165–173° against a neutral of 85°, and TBR at about 175° against 80°. Don't run `PROGRAM_SEQUENCE` on this leg until its neutral angles are measured on the real robot.

## Motion safety

- **Slow, eased moves.** Every move is planned from a speed limit instead of a fixed time. The joint that moves furthest peaks at `MAX_SPEED_DEG_S` (10°/s), and every move starts and ends at zero speed.
- **Hard slew limit.** No pulse width can change faster than `MAX_SLEW_DEG_S` (15°/s), whatever the planner asks for. A command can never jump.
- **Start without jumps.** Servos switch on one at a time. Each one starts at the angle its pot reads at that moment, so its first pulse hardly moves it: only the pot offset, about 2–3°. The servo holds there for 400 ms while the pot offset is learned. If a servo ends up more than 5° away, the start is aborted. Then all joints ease to neutral at the speed limit.
- **Fault cut-out.** If a powered joint's averaged error (after the pot offset) goes above `FAULT_ERR_DEG` (4°, averaged over about 100 ms), all pulses are cut. That covers blocked, stalled and oscillating joints.
- **Pot check.** It refuses to start if a pot reading is out of range or noisy. That catches a disconnected or shorted pot.

## Behaviour

| State | What happens | LED |
|---|---|---|
| After reset | No servo pulses. Feedback is streamed, so you can move a joint by hand to check its pot. | Blue blinking |
| **USER** pressed or `s` sent | Switch-on and easing to neutral as above, then the sequence loops | Green on |
| **USER** pressed or `x` sent | Eases slowly back to neutral, holds 0.3 s, then the pulses stop | Green flashing, then off |
| Fault | All pulses stop | Red on |

**When all legs move, lift the robot off the ground.**

**Front left leg:** its femur neutral (87°, from `Programmed_Gait`) seems to be outside the leg's mechanical range. In an earlier version, which jumped, the femur slammed into its top stop at 107.5°. With this version it would ease slowly towards the stop and the fault cut-out would switch it off. Check the FL neutral angles before you test that leg.

## The sequence

The sequence takes about 57 s per round at 10°/s. The poses are degrees from neutral (body, femur, tibia).

| Keyframes | Poses | Move times | Hold |
|---|---|---|---|
| 1 | neutral | — | 1.0 s |
| 2–4: body alone | +10, −8, 0 | 1.5 s, 2.7 s, 1.2 s | 0.5 s each |
| 5–7: femur alone | +20, −15, 0 | 3.0 s, 5.25 s, 2.25 s | 0.5 s each |
| 8–10: tibia alone | +12, −12, 0 | 1.8 s, 3.6 s, 1.8 s | 0.5 s each |
| 11–18: all three, twice | (+5,+15,−10) (−5,+15,+10) (−5,−10,+10) (+5,−10,−10) | 2.25–3.75 s each | — |
| 19 | neutral | 1.5 s | 1.5 s |

Poses are clamped to the joint limits in `quadruped_control_mcu/Core/Inc/main.h`: body −10/+15°, femur −20/+30°, tibia ±15°.

## Feedback

ADC1 scans all 12 pot inputs continuously over DMA. It uses the same channel order as `quadruped_control_mcu`, where rank n+1 is servo n, with 480-cycle samples. Each channel is the median of the last 64 scans, about 16 ms. It is converted to degrees with the tables in `quadruped_control_mcu/Core/Inc/calibration_ADC.h`, using the mean of the "down" and "up" tables. The channel order was checked on the front-right leg: each pot only changed when its own joint moved. The `IN4`/`IN11` labels in the comments of `calibration_ADC.h` are out of date.

In the front-right test, each pot read about 2.4° below the commanded angle. The firmware now learns this offset for each servo at switch-on, and the error it reports and checks is after that correction.

## Serial output and commands

The ST-LINK virtual COM port runs at 115200 8N1. On this PC it is COM5. Output is interrupt-driven, so it never slows down the motion.

```
  FFL on at 123.2 deg, pot offset -2.4 deg
step 5/19  FL  body  +0  femur +20  tibia +0 deg  move 3000 ms  hold 500 ms  -> FL 1171/979/1164 us
fb 12.3 FL 94.0/91.6/0.0 72.1/69.8/0.1 93.0/90.6/0.0 adc 1341/1060/1320
settled step 05 FL BFL 94.0/91.6/0.0 FFL 67.0/64.5/-0.1 TFL 93.0/90.7/0.1
```

- `fb` lines come every 100 ms. They show the time, then for the body, femur and tibia the sent angle, measured angle and error in degrees (after the pot offset), then the raw ADC codes. A joint without pulses shows `off/<meas>/-`.
- `settled` lines are logged at the end of each hold. `stats` lines come after each round, on stop and on a fault, and include each servo's pot offset.
- Commands: `s` = start, `x` = stop, `a` = also print the raw ADC of all 12 channels, `h` = help.

## Build and flash

Run these from this folder:

```
make            # build/servo_test_leg.elf, .hex, .bin
make flash      # program over ST-LINK with OpenOCD (the board idles until started)
make restore    # put the original board firmware back (../firmware_backup)
```

HAL, CMSIS and `calibration_ADC.h` are taken from `../quadruped_control_mcu`. To use other copies, override them with `make DRIVERS=... CALIB_DIR=...`.

## Changing it

All the settings are at the top of `Src/main.c`:

- `TEST_LEGS`: which legs to drive, as a bit mask, for example `LEG_BIT(LEG_FR)`, `ALL_LEGS`, or `LEG_BIT(LEG_FR) | LEG_BIT(LEG_BL)`.
- `TEST_MODE`: `MODE_TOGETHER` or `MODE_ONE_BY_ONE`. In one-by-one mode, FR, FL, BR and BL run the sequence in turn while the others hold neutral.
- `MAX_SPEED_DEG_S`: how slowly everything moves. `MAX_SLEW_DEG_S` must stay above it.
- `SEQUENCE[]`: the movement. Each line is the pose in degrees from neutral (body, femur, tibia), followed by the hold time in ms. Move times are worked out from the speed limit.
- `FAULT_ERR_DEG`, `FAULT_TAU_MS`, `MAX_OFFSET_DEG`, `SETTLE_MS`, `SEQUENCE_REPEATS`, `JOINT_LIMIT_DEG`.

Neutral angles are the open-loop ("WITHOUT PID") mid points from `Programmed_Gait`, because this firmware commands PWM directly. They are unverified for every leg except front right. PWM timing is the same as in `quadruped_control_mcu`: 1 µs per tick, a 3.1 ms frame, and a pulse of 500 µs + 7.14 µs per degree. Each servo's `dir` sign follows the `joint_range` table in `quadruped_control_mcu`.

| Leg | Joint | Servo | PWM | Feedback | Neutral | dir |
|---|---|---|---|---|---|---|
| FR | body  | 0 BFR  | TIM2 CH1 PA15 | IN4 PA4  | 94° (1171 µs) | + |
| FR | femur | 4 FFR  | TIM3 CH1 PC6  | IN14 PC4 | 87° (1121 µs) | + |
| FR | tibia | 3 TFR  | TIM2 CH4 PB11 | IN11 PC1 | 89° (1136 µs) | + |
| FL | body  | 5 BFL  | TIM3 CH2 PC7  | IN7 PA7  | 94° (1171 µs) | + |
| FL | femur | 6 FFL  | TIM3 CH3 PC8  | IN5 PA5  | 87° (1121 µs) | − |
| FL | tibia | 7 TFL  | TIM3 CH4 PC9  | IN6 PA6  | 93° (1164 µs) | − |
| BR | body  | 1 BBR  | TIM2 CH2 PA1  | IN13 PC3 | 87° (1121 µs) | − |
| BR | femur | 11 FBR | TIM4 CH4 PD15 | IN9 PB1  | 85° (1107 µs) | + |
| BR | tibia | 10 TBR | TIM4 CH3 PD14 | IN12 PC2 | 80° (1071 µs) | + |
| BL | body  | 2 BBL  | TIM2 CH3 PB10 | IN2 PA2  | 86° (1114 µs) | − |
| BL | femur | 8 FBL  | TIM4 CH1 PD12 | IN10 PC0 | 94° (1171 µs) | − |
| BL | tibia | 9 TBL  | TIM4 CH2 PD13 | IN3 PA3  | 96° (1186 µs) | − |
