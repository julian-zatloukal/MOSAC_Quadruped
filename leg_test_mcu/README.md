# Leg test firmware (NUCLEO-F412ZG)

Moves the three servos of **one leg** through a fixed test sequence. The leg can start in **any position**: before any
pulse is sent the firmware reads each joint's potentiometer feedback, starts the PWM exactly at that angle (so nothing
jumps), then ramps slowly to the home pose and loops the sequence. The servos of the other legs get no pulses and stay
limp.

## Select the leg

`Core/Inc/leg_test_config.h`:

```c
#define TEST_LEG LEG_FRONT_RIGHT   // LEG_FRONT_RIGHT, LEG_FRONT_LEFT, LEG_BACK_RIGHT, LEG_BACK_LEFT
```

or from the command line, without editing: `make TEST_LEG=LEG_BACK_LEFT`.

| Leg | Body | Femur | Tibia | Feedback terminal |
|---|---|---|---|---|
| Front-right | BFR, `T2_1` | FFR, `T3_1` | TFR, `T2_4` | J20 |
| Front-left | BFL, `T3_2` | FFL, `T3_3` | TFL, `T3_4` | J21 |
| Back-right | BBR, `T2_2` | FBR, `T4_4` | TBR, `T4_3` | J22 |
| Back-left | BBL, `T2_3` | FBL, `T4_1` | TBL, `T4_2` | J19 |

## Build and flash

Needs `arm-none-eabi-gcc` and `make`; uses the HAL drivers, startup file, linker script and **calibration tables** of
`../quadruped_control_mcu`.

```bash
cd leg_test_mcu
make                      # build/leg_test.elf, .bin, .hex
make flash                # OpenOCD through the ST-LINK micro-USB
```

Or copy `build/leg_test.bin` onto the NUCLEO's USB drive.

Status messages: serial port of the ST-LINK (`/dev/ttyACM0` on Linux) at **115200** baud, e.g.
`picocom -b 115200 /dev/ttyACM0`.

## Use

1. Put the robot **on its stand**, feet off the ground, and switch on the servo power.
2. After reset the leg is **limp**. The terminal prints each joint's reading every 0.5 s: move each joint by hand and
   check that **its own** reading changes. This also checks the feedback wiring.
3. Press the **blue button**:
   - 0.5 s: readings checked (steady, inside the calibration, within 100 deg of home)
   - pulses start at the measured position; 0.4 s check that nothing jumped
   - slow move to home (20 deg/s), then the sequence loops (40 deg/s)
4. Press again: the leg returns home slowly and is **released (limp)**. You can reposition it and start again.

| LED | Meaning |
|---|---|
| Green, slow blink | idle, leg limp |
| Blue, fast blink | checking / moving to or from home |
| Blue, steady | running the sequence |
| Red | fault: leg released, reason on the serial port; press the button to clear |

## Sequence

`Core/Src/sequence.c`, in degrees from home in the leg frame (same convention as the policy, so it means the same
motion on every leg): each joint alone in both directions (body +10/-8, femur +25/-15, tibia +12/-12), then two
combined poses, then home. Values are clipped to the main firmware's limits: body -10..+15, femur -20..+30,
tibia -15..+15.

## Safety checks

| Fault message | Likely cause |
|---|---|
| feedback reads N counts, outside its calibration | feedback wire unplugged or on the wrong terminal (nothing was moved) |
| feedback is not steady | loose or floating feedback wire, or the leg moved during the check (nothing was moved) |
| reads N deg from home | feedback wiring or calibration problem (nothing was moved) |
| moved N deg when its pulses started | feedback belongs to another servo, or the servo is on another PWM header |
| N deg away from its command for 1000 ms | joint blocked, no servo power, or wrong feedback |

Every fault releases the leg. Limits and speeds are in `leg_test_config.h`.

## Notes

- Home angles (`Core/Src/joints.c`) copy the `MID_POINT_*` values of `quadruped_control_mcu/Core/Inc/main.h`: keep them in sync.
- A swapped feedback wire can only be detected once the servo moves: expect a short twitch (about 10 deg) before the
  leg is released.
- Back-right (J22): the firmware reads the tibia on IN12 and the body on IN13, as the main firmware does. The board
  silkscreen shows these two swapped; if the check in step 2 shows them crossed, fix the wiring.
