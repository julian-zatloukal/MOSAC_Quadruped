/*
 * leg_lab configuration. Everything you normally change is in this file.
 *
 * Angles are servo degrees on the PWM scale, the same scale as calibration_ADC.h
 * and quadruped_control_mcu: pulse = 500 us + deg * 1000/140 us.
 * "q" is the joint angle in the Webots convention: servo deg = mid point + dir * q.
 *
 * Every setting is wrapped in #ifndef so it can also be overridden from the
 * command line, e.g.  make PROGRAM=PROGRAM_LIMITS TEST_LEG=LEG_BR
 */
#ifndef LEG_LAB_CONFIG_H
#define LEG_LAB_CONFIG_H

/* ================================ Names ================================ */

#define PROGRAM_MONITOR 0   /* never sends a pulse: pots, hand ranges ('r'), calibrator pose ('c') */
#define PROGRAM_CHECK   1   /* one servo at a time: starts where it is, small slow sweeps          */
#define PROGRAM_POTCAL  2   /* one servo with its horn OFF: full pulse sweep -> new pot table      */
#define PROGRAM_LIMITS  3   /* leg lifted: guarded search of every joint limit + coupled maps      */

#define LEG_FR 0
#define LEG_FL 1
#define LEG_BR 2
#define LEG_BL 3

#define JOINT_BODY  0       /* abduction (hip) */
#define JOINT_FEMUR 1
#define JOINT_TIBIA 2       /* driven through the four-bar linkage */

#define POT_TABLE     0     /* use this servo position's table in calibration_ADC.h            */
#define POT_LINE      1     /* use a straight line adc = a + b * deg (printed by PROGRAM_POTCAL) */
#define POT_UNTRUSTED 2     /* the reading does not match this servo: it is never switched on,
                               except by PROGRAM_POTCAL                                         */

#define UNSET (-999.0f)     /* "not measured yet" in the tables below */

/* ============================ What to run ============================== */

#ifndef PROGRAM
#define PROGRAM   PROGRAM_MONITOR
#endif
#ifndef TEST_LEG
#define TEST_LEG  LEG_FR    /* only this leg's PWM pins are ever connected */
#endif

/* --- PROGRAM_CHECK --- */
#ifndef CHECK_JOINTS
#define CHECK_JOINTS        0x7u            /* bits (1 << JOINT_x); 0x7 = all three */
#endif
#ifndef CHECK_AMPLITUDES
#define CHECK_AMPLITUDES    { 5.0f, 10.0f } /* sweeps +A / -A around where the joint is */
#endif
#define CHECK_SPEED_DEG_S   5.0f
#define CHECK_HOLD_MS       400u
#define CHECK_GAIN_MIN      0.8f            /* pot change / commanded change on the largest sweep */
#define CHECK_GAIN_MAX      1.2f
#define CHECK_BACKLASH_MAX  2.0f            /* deg, approach from above minus from below */

/* --- PROGRAM_POTCAL ---
 * Take the horn (or the link) OFF the servo shaft first: the shaft then turns
 * freely over the whole 0..266 deg range and nothing can be hit. */
#ifndef POTCAL_JOINT
#define POTCAL_JOINT        JOINT_FEMUR
#endif
#ifndef POTCAL_HORN_REMOVED
#define POTCAL_HORN_REMOVED 0       /* set to 1 to confirm the horn is off; the program refuses otherwise */
#endif
#ifndef POTCAL_MOUNT_DEG
#define POTCAL_MOUNT_DEG    UNSET   /* at the end, hold this angle until USER so the link can be pushed on
                                       (e.g. 134.8, the old FFL calibrator reading); UNSET = go limp */
#endif
#define POTCAL_SPEED_DEG_S  10.0f
#define POTCAL_HOLD_MS      800u    /* settle at each table angle before reading */
#define POTCAL_MIN_FOLLOW   0.3f    /* pot change below this share of the expected one = not following */

/* --- PROGRAM_LIMITS (robot lifted, the foot must not touch anything) --- */
#ifndef LIMITS_JOINTS
#define LIMITS_JOINTS       0x7u    /* joints whose own limits are searched */
#endif
#ifndef LIMITS_REFINE
#define LIMITS_REFINE       1       /* second, slower approach to each contact (repeatability) */
#endif
#ifndef LIMITS_MAP_FT
#define LIMITS_MAP_FT       1       /* tibia limits at several femur angles */
#endif
#ifndef LIMITS_MAP_FB
#define LIMITS_MAP_FB       1       /* body limits at femur low / start / high */
#endif
#ifndef MAP_FEMUR_STEP_DEG
#define MAP_FEMUR_STEP_DEG  15.0f
#endif
#ifndef MAP_MARGIN_DEG
#define MAP_MARGIN_DEG      5.0f    /* stay this far inside a measured limit when moving there */
#endif

/* ========================= Per-servo settings ==========================
 * Servo order: 0 BFR  1 BBR  2 BBL  3 TFR  4 FFR  5 BFL  6 FFL  7 TFL  8 FBL  9 TBL  10 TBR  11 FBR */

/* How each pot reading is turned into degrees. FFL is untrusted: the femur servo
 * was swapped, and the table at that position (SERVO 04) does not fit it. Run
 * PROGRAM_POTCAL on it, then set POT_LINE (with its numbers below) or paste the
 * new table into calibration_ADC.h and set POT_TABLE. */
#ifndef CFG_POT_MODEL
#define CFG_POT_MODEL { \
    POT_TABLE, POT_TABLE, POT_TABLE, POT_TABLE, POT_TABLE, POT_TABLE, \
    POT_UNTRUSTED, POT_TABLE, POT_TABLE, POT_TABLE, POT_TABLE, POT_TABLE }
#endif

/* POT_LINE parameters { a = ADC code at 0 deg, b = ADC codes per deg }. */
#ifndef CFG_POT_LINE
#define CFG_POT_LINE { \
    {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0} }
#endif

/* Expected pot offset (pot reading - command) used for the very first pulse, so
 * the servo hardly moves when it switches on. From the 2026-11-17 checks. The
 * real offset is measured again at every switch-on. */
#ifndef CFG_OFFSET_HINT
#define CFG_OFFSET_HINT { \
    -2.4f, -2.5f, -1.8f, -2.4f, -2.4f, -0.1f, 0.0f, 0.0f, -2.6f, -0.8f, -2.9f, -0.6f }
#endif

/* Range of each pot when the joint is moved BY HAND with the servos unpowered
 * (PROGRAM_MONITOR, 'r'), { min, max } in pot degrees. Powered searches never go
 * more than HAND_MARGIN_DEG beyond it. Strongly recommended for every joint;
 * required for the tibia unless the calibrator pose is set. */
#ifndef CFG_HAND_RANGE
#define CFG_HAND_RANGE { \
    {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET}, \
    {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET}, {UNSET,UNSET} }
#endif

/* Pot reading (deg) with the printed calibrator fitted (PROGRAM_MONITOR, 'c').
 * Gives the joint angle q of every reading, and with it the tibia linkage guard.
 * For comparison, the calibrator readings behind the PID mid points in
 * quadruped_control_mcu/Core/Inc/main.h were: BFR 96.1 BBR 80.5 BBL 85.5 TFR 69.0
 * FFR 71.9 BFL 93.9 FFL 134.8 (old servo) TFL 120.1 FBL 130.4 TBL 122.6 TBR 55.4
 * FBR 55.9. Measure again before relying on them. */
#ifndef CFG_REF_POSE
#define CFG_REF_POSE { \
    UNSET, UNSET, UNSET, UNSET, UNSET, UNSET, UNSET, UNSET, UNSET, UNSET, UNSET, UNSET }
#endif

/* Joint angle q at the calibrator pose (main.h DELTA_REF_*). gait_core.py uses
 * 16.75 instead of 18.35 for the tibia; see the README. */
#ifndef REF_Q
#define REF_Q { 0.0f, -42.65021f, -18.34979f }   /* body, femur, tibia */
#endif

/* Tibia four-bar linkage (A 24, B 40.5, C 28.66, D 27 mm): the q range where both
 * transmission angles stay >= 30 deg. Towards higher q the crank binds at about
 * q = +89 (coupler and rocker in line); towards lower q the coupler folds onto the
 * crank at about q = -130, where the knee stops and reverses while the servo keeps
 * turning. Used only when the calibrator pose of the tibia is set. */
#ifndef TIBIA_Q_SAFE
#define TIBIA_Q_SAFE { -63.0f, 69.0f }
#endif

/* ============================ Motion safety ============================ */

#define TICK_MS             10u     /* control loop period                                     */
#define MAX_SPEED_DEG_S     8.0f    /* peak speed of every planned (eased) move                 */
#define MAX_SLEW_DEG_S      12.0f   /* hard limit on how fast any sent command can change       */
#define MIN_MOVE_MS         400u    /* shortest planned move                                    */
#define ABS_MIN_DEG         5.0f    /* no command outside this range is ever sent               */
#define ABS_MAX_DEG         261.0f

/* Switch-on: first pulse where the pot says the servo is, watched at 1 ms. */
#define PROBE_MS            300u
#define PROBE_CUT_DEG       3.0f    /* a pot of the leg this far off ...                        */
#define PROBE_SUSTAIN_MS    5u      /* ... for this long cuts the servo at once                 */
#define PROBE_FAST_DEG      6.0f    /* or this far off for PROBE_FAST_MS                        */
#define PROBE_FAST_MS       2u
#define PROBE_GLITCH_DEG_MS 1.5f    /* a reading changing faster than this per ms is electrical
                                       (a servo turns at most ~0.5 deg/ms): ignored, as the
                                       pot dips seen when a motor starts                        */
#define SETTLE_MS           400u    /* hold, then the pot offset is measured                    */
#define MAX_OFFSET_DEG      5.0f    /* a servo further off than this does not hold where its pot says */
#define WIGGLE_DEG          2.0f    /* then +-2 deg: the pot must follow                        */
#define WIGGLE_MIN_GAIN     0.5f
#define WIGGLE_SPEED_DEG_S  2.0f

/* Normal moves */
#define MOVE_BLOCK_DEG      3.0f    /* a joint this far behind ...                              */
#define MOVE_BLOCK_MS       150u    /* ... for this long: stop pushing, abort (no cut)          */
#define FAULT_ERR_DEG       4.5f    /* averaged error above this cuts all pulses (last resort)  */
#define FAULT_TAU_MS        100.0f
#define EXEMPT_ABORT_DEG    8.0f    /* same, for a joint whose error the program watches itself */
#define FREEZE_MAX_DEG      6.0f    /* a freeze never moves the command further than this       */
#define PARK_HOLD_MS        300u

/* Limit searches */
#define SEARCH_SPEED_DEG_S  4.0f    /* towards a limit                                          */
#define SEARCH_SLOW_DEG_S   1.0f    /* after the first sign of contact                          */
#define REFINE_SPEED_DEG_S  1.5f
#define SEARCH_RAMP_S       0.5f    /* speed ramp at the start of a search                      */
#define SEARCH_BASE_S       0.6f    /* lag baseline learned during the first part of a search   */
#define SEARCH_BASE_TAU_S   1.0f
#define SEARCH_ONSET_DEG    1.0f    /* lag above the baseline where contact is taken to start  */
#define SEARCH_DETECT_DEG   2.0f    /* lag above the baseline ...                               */
#define REFINE_DETECT_DEG   1.5f
#define SEARCH_DETECT_MS    100u    /* ... for this long: contact                               */
#define SEARCH_FAST_DEG     4.0f    /* immediate contact                                        */
#define SEARCH_AHEAD_DEG    3.0f    /* joint suddenly ahead of its command: snap-through        */
#define HELD_DEFLECT_DEG    2.5f    /* a held joint pushed this far: contact through the leg    */
#define HELD_DEFLECT_MS     100u
#define SEARCH_CAP_WAIT_MS  500u
#define SEARCH_BACKOFF_DEG  4.0f
#define LIMIT_TRAVEL_DEG    { 40.0f, 100.0f, 45.0f }   /* body, femur, tibia: max from the start */
#define HAND_MARGIN_DEG     3.0f

/* Electrical watch: pots of the unpowered legs must stay still, VDDA must hold. */
#define REF_PAUSE_DEG       2.0f    /* reference pots shifted: pause, end a search              */
#define REF_ABORT_DEG       6.0f    /* cut at once                                              */
#define REF_CLEAR_DEG       1.0f
#define FB_COMMON_MODE      1       /* subtract the median shift of the reference pots from the
                                       leg's pots (ground shift from servo current)             */
#define SEARCH_REF_DEG      1.0f    /* reference pots shifting this much during a search means
                                       current draw: taken as a stall (contact)                 */
#define DISTURB_MAX_MS      1000u   /* a pause longer than this cuts                            */
#define VDDA_MIN_V          3.0f    /* cut below this (the Nucleo is about to brown out)        */
#define VDDA_DIP_V          0.05f   /* pause on a dip this big                                  */

/* Feedback */
#define FB_MAX_IQR          40u     /* pot trusted only if its ADC spread is below this         */
#define TELEMETRY_MS        200u
#define IDLE_STREAM_MS      500u

#endif /* LEG_LAB_CONFIG_H */
