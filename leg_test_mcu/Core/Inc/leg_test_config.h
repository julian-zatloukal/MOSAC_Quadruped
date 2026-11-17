#ifndef LEG_TEST_CONFIG_H
#define LEG_TEST_CONFIG_H

/*
 * Leg under test. One of:
 *   LEG_FRONT_RIGHT, LEG_FRONT_LEFT, LEG_BACK_RIGHT, LEG_BACK_LEFT
 * Can also be set from the command line: make TEST_LEG=LEG_BACK_LEFT
 */
#ifndef TEST_LEG
#define TEST_LEG LEG_FRONT_RIGHT
#endif

/* ---- Everything below has safe defaults; change only if needed ---- */

// Joint speeds (deg/s). The trajectory never exceeds these.
#define HOMING_SPEED_DEG_S          20.0f   // from the starting position to home, and back to home on stop
#define SEQUENCE_SPEED_DEG_S        40.0f   // between waypoints of the test sequence

// Start-up checks, done with the servos limp before any pulse is sent
#define MEASURE_TIME_MS             500     // how long the feedback is watched before engaging
#define MEASURE_MAX_SPREAD_DEG      3.0f    // reading must be this steady (detects a floating/unplugged feedback wire)
#define START_MAX_FROM_HOME_DEG     100.0f  // a reading further than this from home is treated as a feedback error

// Engage: the PWM starts at the measured angle, so the joint should not move
#define ENGAGE_SETTLE_MS            400
#define ENGAGE_MAX_JUMP_DEG         10.0f   // larger jump = the feedback does not belong to this servo -> release

// While moving: a joint that cannot follow its command is released
#define TRACK_MAX_ERR_DEG           20.0f
#define TRACK_MAX_ERR_MS            1000

#define CONTROL_PERIOD_MS           10      // control loop period
#define LOG_PERIOD_MS               200     // UART status line period while moving
#define IDLE_LOG_PERIOD_MS          500     // UART status line period while limp
#define LOG_BAUDRATE                115200  // ST-LINK virtual COM port

#endif
