/* Motion core for the three joints of TEST_LEG: slew-limited output, safe
 * switch-on, guarded moves, limit searches, parking, and the safety watch that
 * runs every tick.
 *
 * Two ways to stop:
 *   freeze  the joint is commanded where it actually is (it stops pushing) and
 *           keeps holding. Used for contacts and blocks: nothing drops.
 *   cut     no more pulses: the servo goes limp, so a lifted leg falls. Used only
 *           for emergencies (supply, a joint not following, a stuck search) and
 *           at the very end, once the joint is back at its rest pose.
 */
#ifndef LEG_LAB_MOTION_H
#define LEG_LAB_MOTION_H

#include <stdbool.h>
#include <stdint.h>
#include "robot.h"

typedef struct {
    uint8_t  servo;
    bool     on;            /* pulses being sent                                     */
    float    cmd;           /* planned command, deg                                  */
    float    out;           /* sent command (follows cmd at MAX_SLEW_DEG_S at most)   */
    float    offset;        /* pot - out, measured at switch-on                      */
    bool     offset_known;
    float    rest;          /* command at switch-on: the pose the limp leg rested in */
    float    lo, hi;        /* window: no command outside it is ever sent            */
    bool     exempt;        /* error watched by the caller (searches, potcal)        */
    float    err_avg;
    uint32_t invalid_ms;
} joint_t;

extern joint_t mo_j[NJ];
extern float   mo_track_max[NJ];      /* largest |error| seen during mo_move(), reset by the caller */

typedef enum { MO_OK = 0, MO_PAUSE, MO_STOP, MO_CUT } mo_tick_t;
typedef enum { R_OK = 0, R_STOPPED, R_CUT, R_BLOCKED, R_REFUSED } result_t;

typedef enum {
    S_CONTACT,          /* the joint itself was blocked                       */
    S_CONTACT_OTHER,    /* a held joint of the leg was pushed                 */
    S_AHEAD,            /* the joint ran ahead of its command (snap-through)  */
    S_CAP,              /* reached the edge of its window without contact     */
    S_ELECTRICAL,       /* supply / ground disturbance (often a stall)        */
    S_STOPPED,
    S_CUT,
} search_kind_t;

typedef struct {
    search_kind_t kind;
    float onset;        /* where contact began (command deg = pot - offset) */
    float at_end;       /* where the joint was when the search ended        */
    int   other;        /* joint that was pushed, for S_CONTACT_OTHER       */
    float peak_dev;     /* largest lag above the baseline, deg              */
} search_t;

extern const char *const SEARCH_KIND_NAME[];

void      mo_init(void);
void      mo_begin_program(void);         /* clears stop requests and the last-stop text */
void      mo_end_program(void);
mo_tick_t mo_tick(void);                  /* one TICK_MS step: input, feedback, safety, outputs */
int       mo_char(void);                  /* character received in the last tick, 0 if none     */
bool      mo_pressed(void);               /* USER pressed in the last tick                      */

float     mo_meas(int j);                 /* where the joint is, in command degrees (pot - offset) */
float     mo_err(int j);                  /* meas - sent                                        */
uint16_t  mo_us(float deg);

result_t  mo_switch_on(int j);            /* probe + settle + wiggle; joint then holds there */
result_t  mo_move(const float to[NJ], float speed, uint32_t hold_ms);
result_t  mo_move_joint(int j, float to, float speed, uint32_t hold_ms);
result_t  mo_hold(uint32_t ms);
search_t  mo_search(int j, int dir, float speed, float detect_deg);
void      mo_freeze(int j);
void      mo_freeze_all(void);
void      mo_set_window(int j, float lo, float hi);   /* always keeps the current command inside */
result_t  mo_park_joint(int j);           /* back to its rest pose, then cut */
result_t  mo_park(void);                  /* all powered joints, then cut    */
void      mo_cut(int j);
void      mo_cut_all(const char *why, int servo);
bool      mo_any_on(void);
bool      mo_was_cut(void);

void        mo_note_stop(const char *why, int servo);   /* servo < 0: none */
const char *mo_last_stop(void);
void        mo_telemetry_now(void);

#endif
