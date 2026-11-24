/* The walk as the board runs it (walk_*), and its dry run (plan_walk): the whole
 * walk, ramp and stop included, computed before a servo moves. No hardware
 * access: also built on the host (tools/plan_host.c). */
#ifndef GAIT_RUNNER_PLAN_H
#define GAIT_RUNNER_PLAN_H

#include <stdbool.h>
#include "gait.h"
#include "robot.h"

/* One walk: ramp, `cycles` cycles at the stride, then a stop and the wait until
 * every foot is centred. One walk_tick() per control tick. */
typedef struct {
    double dt;          /* gait time per tick = TICK * speed scale */
    double t_stop;      /* gait time at which the walk stops by itself */
    bool   stopping;
} walk_t;

void walk_begin(walk_t *w, gait_t *g, float speed, float stride, int cycles);
void walk_stop(walk_t *w, gait_t *g);
bool walk_tick(walk_t *w, gait_t *g);   /* true once stopped and centred */

typedef struct {
    bool        ok;
    const char *why;                    /* if not ok */
    int         why_servo;              /* -1 if none */
    float       why_q;
    float       speed;                  /* speed scale the walk uses (may be below the one asked for) */
    bool        slowed;                 /* lowered to keep under MAX_JOINT_SPEED_DEG_S */
    float       peak50;                 /* fastest joint, deg/s over 50 ms, at that speed */
    float       peak_tick;              /* fastest joint over one tick */
    float       q_lo[SERVO_COUNT], q_hi[SERVO_COUNT];
    float       deg_lo[SERVO_COUNT], deg_hi[SERVO_COUNT];
    float       seconds;                /* start to stopped and centred */
    long        ticks;
} plan_t;

/* Dry run of a walk from the state `now`. yield (may be 0) is called every 64
 * steps, so the board keeps its safety watch running meanwhile. */
bool plan_walk(const gait_t *now, float speed, float stride, int cycles, plan_t *pl, void (*yield)(void));

#endif
