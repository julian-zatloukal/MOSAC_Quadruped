/*
 * gait_runner configuration. Everything you normally change is in this file.
 *
 * "servo deg" is the PWM scale of calibration_ADC.h, quadruped_control_mcu and
 * leg_lab: pulse = 500 us + deg * 1000/140 us.
 * "q" is the joint angle of the Webots model, the angle gait_core.py computes:
 *     servo deg = mid point + dir * q,   mid point = calibrator reading - dir * REF_Q + trim
 *
 * Every setting is wrapped in #ifndef, so it can also be given to make, e.g.
 *     ./build.sh RUN_MODE=RUN_STAND
 */
#ifndef GAIT_RUNNER_CONFIG_H
#define GAIT_RUNNER_CONFIG_H

/* ================================ Names ================================ */

#define RUN_MONITOR 0   /* no pulse can be sent: pots, joint angles, calibrator capture ('c') */
#define RUN_STAND   1   /* power up, stand up, hold; USER sits down. No walking.              */
#define RUN_WALK    2   /* the same, and USER while standing walks                             */

#define POT_TABLE     0 /* this servo position's table in calibration_ADC.h                   */
#define POT_LINE      1 /* adc = a + b * deg, from leg_lab PROGRAM_POTCAL                      */
#define POT_UNTRUSTED 2 /* the reading does not match the servo: gait_runner refuses to start */

#define UNSET (-999.0f)

/* ============================ What to run ============================== */

#ifndef RUN_MODE
#define RUN_MODE    RUN_MONITOR         /* a fresh build never moves anything */
#endif

/* --- the walk (RUN_WALK) --- */
#ifndef SPEED_SCALE
#define SPEED_SCALE 0.5f    /* gait clock against Webots: 1.0 = the simulated walk (period 1.76 s),
                               0.5 = half as fast (period 3.53 s). '+' / '-' change it while standing */
#endif
#ifndef STRIDE
#define STRIDE      1.0f    /* fraction of the full stride (70.9 mm), the "forward" of the teleop page */
#endif
#ifndef WALK_CYCLES
#define WALK_CYCLES 4       /* gait cycles at full stride after the ramp; then the walk stops by itself */
#endif
#define SPEED_MIN   0.1f
#define SPEED_MAX   1.0f
#define SPEED_STEP  0.1f
#define MAX_JOINT_SPEED_DEG_S 250.0f /* fastest joint motion (over 50 ms, as tune_gait.py) a walk may
                                        command; the speed scale is lowered to stay under it. The
                                        simulated walk needs ~345 at SPEED_SCALE 1. A DS3235 does
                                        ~400-460 with no load; nothing was measured under load.   */
#define SLEW_LIMIT_DEG_S      400.0f /* hard limit on every command change while walking            */

/* --- standing up and sitting down --- */
#define H_LOW_MM          110.0f    /* crouch: hip-to-foot height the robot rises from and sits down to */
#define H_RATE_MM_S       12.0f     /* peak speed of rising and sitting down                     */
#define MOVE_SPEED_DEG_S  10.0f     /* peak speed of joint moves (power-up pose -> crouch, back to stand) */
#define MOVE_SLEW_DEG_S   20.0f     /* hard limit on every command change outside the walk       */
#define STAND_SETTLE_MS   1500u     /* then the stand check                                      */
#define STAND_ERR_DEG     5.0f      /* a joint further than this from its command fails the check */

/* ========================= Per-servo calibration ======================== *
 * Servo order: 0 BFR  1 BBR  2 BBL  3 TFR  4 FFR  5 BFL  6 FFL  7 TFL  8 FBL  9 TBL  10 TBR  11 FBR
 * The pot settings have the same names and format as in leg_lab/Src/config.h:
 * copy the lines from there once they are measured. */

/* FFL is untrusted: its femur servo was swapped. Run leg_lab PROGRAM_POTCAL on it
 * (horn off), then set POT_LINE and its numbers here. */
#ifndef CFG_POT_MODEL
#define CFG_POT_MODEL { \
    POT_TABLE, POT_TABLE, POT_TABLE, POT_TABLE, POT_TABLE, POT_TABLE, \
    POT_UNTRUSTED, POT_TABLE, POT_TABLE, POT_TABLE, POT_TABLE, POT_TABLE }
#endif
#ifndef CFG_POT_LINE
#define CFG_POT_LINE { \
    {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0} }
#endif

/* Expected pot offset (pot - command) for the first pulse; measured again at every
 * switch-on. From the 2026-11-17 checks. */
#ifndef CFG_OFFSET_HINT
#define CFG_OFFSET_HINT { \
    -2.4f, -2.5f, -1.8f, -2.4f, -2.4f, -0.1f, 0.0f, 0.0f, -2.6f, -0.8f, -2.9f, -0.6f }
#endif

/* Pot reading (pot deg) of every joint with the printed calibrators fitted:
 * RUN_MONITOR 'c' here, or leg_lab PROGRAM_MONITOR 'c' (CFG_REF_POSE there).
 * Defaults: the readings behind the mid points of quadruped_control_mcu main.h,
 * from an older calibration. FFL is UNSET: that reading was of the old servo.
 * Capture them all again before the first powered run. */
#ifndef CFG_CAL_POSE
#define CFG_CAL_POSE { \
    96.13f, 80.50f, 85.49f, 69.01f, 71.90f, 93.95f, \
    UNSET, 120.14f, 130.44f, 122.62f, 55.43f, 55.87f }
#endif

/* Correction added to each mid point (servo deg), e.g. from an inclinometer on the
 * standing robot. */
#ifndef CFG_TRIM
#define CFG_TRIM { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#endif

/* Joint angle q at the calibrator pose (body, femur, tibia): DELTA_REF_* of
 * quadruped_control_mcu main.h, as leg_lab. gait_core.py has 16.75 instead of
 * 18.35 for the tibia (1.6 deg); which one the calibrator really makes is open. */
#ifndef REF_Q
#define REF_Q { 0.0f, -42.65021f, -18.34979f }
#endif

/* Joint limits measured with leg_lab PROGRAM_LIMITS: the command (servo deg, the
 * "cmd" of its RES,limit lines) at the low and the high contact. No command closer
 * than LIMIT_MARGIN_DEG to a measured contact is sent. */
#ifndef CFG_CMD_LIMITS
#define CFG_CMD_LIMITS { \
    {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET}, \
    {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET} }
#endif
#define LIMIT_MARGIN_DEG  5.0f

/* ============================ Joint windows ============================ *
 * Every pose and gait target must have its q inside these (body, femur, tibia).
 * The walk needs femur -18..21 and tibia -31..17 (ramp and stop included), the
 * crouch at 110 mm femur -10, tibia +10. */
#define Q_WINDOW        { { -5.0f, 5.0f }, { -30.0f, 40.0f }, { -50.0f, 30.0f } }
/* Tibia four-bar linkage: both transmission angles stay >= 30 deg in this q range
 * (binds near +89, folds near -130). Also the range a tibia may be in at power-up. */
#define TIBIA_Q_SAFE    { -63.0f, 69.0f }
#define START_MARGIN_DEG 25.0f  /* at power-up a joint may be this far outside its window */
#define ABS_MIN_DEG       5.0f  /* no pulse outside this servo deg range is ever sent */
#define ABS_MAX_DEG     261.0f

/* ============================ Motion safety ============================ */

#define TICK_MS             10u

/* Switch-on, one servo at a time: first pulse where the pot says the servo is. */
#define PROBE_MS            300u
#define PROBE_CUT_DEG       3.0f    /* a pot of the leg this far off ...                   */
#define PROBE_SUSTAIN_MS    5u      /* ... for this long cuts everything at once          */
#define PROBE_FAST_DEG      6.0f    /* or this far off for PROBE_FAST_MS                   */
#define PROBE_FAST_MS       2u
#define PROBE_GLITCH_DEG_MS 1.5f    /* faster than a servo can turn: electrical, ignored   */
#define SETTLE_MS           400u
#define MAX_OFFSET_DEG      5.0f    /* holds further than this from where its pot said: cut */
#ifndef POWERUP_WIGGLE
#define POWERUP_WIGGLE      1       /* +-WIGGLE_DEG after each switch-on: the pot must follow */
#endif
#define WIGGLE_DEG          2.0f
#define WIGGLE_SPEED_DEG_S  8.0f
#define WIGGLE_MIN_GAIN     0.5f
#define MIN_MOVE_MS         400u

/* Watch, every tick, every powered servo */
#define TRACK_TAU_MS        100.0f  /* error averaging                                        */
#define TRACK_MOVE_DEG      6.0f    /* averaged error limit while standing and moving slowly */
#define TRACK_WALK_DEG      15.0f   /* while walking (a servo lags a moving command)         */
#define POT_INVALID_MS      100u    /* pot reading invalid this long: fault                  */
#define FREEZE_MAX_DEG      6.0f    /* a freeze moves a command at most this far, to where
                                       the joint is, so a blocked servo stops pushing        */
#define YIELD_FROM_DEG      2.0f    /* after a fault, a joint this far off its command ...    */
#define YIELD_DEG_S         30.0f   /* ... has the command moved towards it this fast ...     */
#define YIELD_MAX_DEG       25.0f   /* ... and at most this far, so it stops pushing          */
#define STALL_DEG           8.0f    /* a joint still this far off ...                         */
#define STALL_MS            1000u   /* ... for this long: every pulse is cut                  */

/* Electrical watch: a ground shift moves every pot reading the same way (seen on
 * 2026-11-24 before a brown-out). The median error of all powered servos is that
 * shift; it, or a VDDA dip, pauses everything (the commands hold). */
#define CM_PAUSE_DEG        4.0f
#define CM_CLEAR_DEG        2.0f
#define VDDA_DIP_V          0.05f
#define PAUSE_MAX_MS        1000u   /* a pause longer than this is a fault                  */
#define VDDA_MIN_V          3.0f    /* below this every pulse is cut (the Nucleo is about to
                                       brown out, which would drop them anyway)              */

/* Feedback */
#define FB_MAX_IQR          40u     /* pot trusted only if its ADC spread is below this     */
#define TELEMETRY_MS        200u
#define IDLE_STREAM_MS      1000u

#endif /* GAIT_RUNNER_CONFIG_H */
