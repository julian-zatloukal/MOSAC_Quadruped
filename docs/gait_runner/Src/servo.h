/* The servos: 10 ms control tick, the safety watch, switch-on, eased moves,
 * freeze and cut. Every pulse the firmware sends goes through sv_tick(). */
#ifndef GAIT_RUNNER_SERVO_H
#define GAIT_RUNNER_SERVO_H

#include <stdbool.h>
#include <stdint.h>
#include "robot.h"

typedef enum { TK_OK, TK_PAUSE, TK_FAULT, TK_CUT } tk_t;
typedef enum { R_OK, R_REFUSED, R_FAULT, R_FROZEN, R_CUT } result_t;

typedef struct {
    bool     on;
    float    cmd;           /* servo deg wanted                           */
    float    out;           /* servo deg sent (cmd through the slew limit) */
    float    offset;        /* pot - command, measured at switch-on       */
    bool     offset_known;
    float    err_avg;       /* averaged |error|                          */
    uint32_t invalid_ms;
} servo_t;

extern servo_t sv[SERVO_COUNT];

void     sv_init(void);
void     sv_begin(void);                 /* new session: clears the cut / fault state */

/* One control tick: waits for it, reads the pots, runs the watch, sends the
 * pulses. 'k' cuts everything in any state. */
tk_t     sv_tick(void);
void     sv_poll(void);                  /* runs a tick only if one is due (never waits) */
int      sv_take_char(void);             /* last key received, 0 if none (then cleared) */
bool     sv_take_press(void);            /* USER pressed since the last call            */
bool     sv_user_stop(void);             /* USER or 'x' since the last call: freezes, true */

void     sv_set_limits(float track_deg, float slew_deg_s);
void     sv_set_state(char state, double gait_t);   /* for the telemetry */

bool     sv_any_on(void);
bool     sv_all_on(void);
bool     sv_faulted(void);
bool     sv_paused(void);
const char *sv_last_stop(void);
float    sv_meas(int n);                 /* pot - offset: where the servo is, on the command scale */
float    sv_err(int n);                  /* sv_meas - out */

void     sv_command(int n, float deg);   /* new target; ignored while faulted */
void     sv_fault(const char *why, int servo);  /* freeze, hold, report */
void     sv_clear_fault(void);
void     sv_freeze_all(void);            /* stop moving: every command to where its joint is */
void     sv_freeze_by_user(void);        /* the same, reported as a user stop */
void     sv_cut_all(const char *why);

result_t sv_power_up(void);              /* every servo, one at a time, where it is */
result_t sv_move(const float to[SERVO_COUNT], float speed_deg_s);   /* eased, all together */
result_t sv_hold(uint32_t ms);
result_t sv_fault_hold(void);            /* after a fault: blocked joints give way; cut if one stays stalled */

#endif
