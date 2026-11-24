#include "plan.h"
#include "config.h"
#include "pose.h"

#define TICK_S   ((double)TICK_MS / 1000.0)
#define WINDOW_TICKS 5                  /* 50 ms, the SPEED_WINDOW of gait_core.py */
#define MAX_TICKS    200000L

static float absf(float v) { return v < 0.0f ? -v : v; }

void walk_begin(walk_t *w, gait_t *g, float speed, float stride, int cycles)
{
    w->dt       = TICK_S * (double)speed;
    w->t_stop   = g->t + (g->p.ramp_cycles + (double)cycles) * g->p.period;
    w->stopping = false;
    gait_drive(g, stride);
}

void walk_stop(walk_t *w, gait_t *g)
{
    if (w->stopping) return;
    gait_stop(g);
    w->stopping = true;
}

bool walk_tick(walk_t *w, gait_t *g)
{
    if (!w->stopping && g->t >= w->t_stop) walk_stop(w, g);
    gait_tick(g, w->dt);
    return w->stopping && g->idle;
}

static bool run(const gait_t *now, float speed, float stride, int cycles, plan_t *pl, void (*yield)(void))
{
    gait_t g = *now;
    walk_t w;
    float  q[SERVO_COUNT], hist[WINDOW_TICKS][SERVO_COUNT];

    pl->ok = false;
    pl->why = 0;
    pl->why_servo = -1;
    pl->why_q = 0.0f;
    pl->peak50 = pl->peak_tick = 0.0f;
    pose_of_gait(&g, q);
    for (int n = 0; n < SERVO_COUNT; n++) {
        pl->q_lo[n] = pl->q_hi[n] = q[n];
        pl->deg_lo[n] = pl->deg_hi[n] = pose_deg(n, q[n]);
        for (int i = 0; i < WINDOW_TICKS; i++) hist[i][n] = q[n];
    }

    walk_begin(&w, &g, speed, stride, cycles);
    for (long i = 1; i <= MAX_TICKS; i++) {
        bool done = walk_tick(&w, &g);
        if (!g.ik_ok) { pl->why = "a foot goes out of reach"; return false; }
        pose_of_gait(&g, q);
        float *old = hist[i % WINDOW_TICKS], *prev = hist[(i - 1) % WINDOW_TICKS];
        for (int n = 0; n < SERVO_COUNT; n++) {
            float deg;
            const char *why = pose_check(n, q[n], &deg);
            if (why) { pl->why = why; pl->why_servo = n; pl->why_q = q[n]; return false; }
            if (q[n] < pl->q_lo[n]) pl->q_lo[n] = q[n];
            if (q[n] > pl->q_hi[n]) pl->q_hi[n] = q[n];
            if (deg < pl->deg_lo[n]) pl->deg_lo[n] = deg;
            if (deg > pl->deg_hi[n]) pl->deg_hi[n] = deg;
            float v50 = absf(q[n] - old[n]) / (float)(WINDOW_TICKS * TICK_S);
            float v1  = absf(q[n] - prev[n]) / (float)TICK_S;
            if (v50 > pl->peak50) pl->peak50 = v50;
            if (v1 > pl->peak_tick) pl->peak_tick = v1;
            old[n] = q[n];          /* the oldest entry becomes the newest */
        }
        if (yield && (i & 63) == 0) yield();      /* ~4 ms of work on the board */
        if (done) {
            pl->ticks = i;
            pl->seconds = (float)((double)i * TICK_S);
            pl->ok = true;
            return true;
        }
    }
    pl->why = "the walk did not end";
    return false;
}

bool plan_walk(const gait_t *now, float speed, float stride, int cycles, plan_t *pl, void (*yield)(void))
{
    if (speed > SPEED_MAX) speed = SPEED_MAX;
    pl->slowed = false;
    if (!(speed >= SPEED_MIN)) { pl->ok = false; pl->why = "speed scale below SPEED_MIN"; pl->why_servo = -1; return false; }
    if (!(stride > 0.0f && stride <= 1.0f)) { pl->ok = false; pl->why = "STRIDE must be in (0, 1]"; pl->why_servo = -1; return false; }
    if (cycles < 1 || cycles > 12) { pl->ok = false; pl->why = "WALK_CYCLES must be 1..12"; pl->why_servo = -1; return false; }

    for (int iter = 0; iter < 5; iter++) {
        if (!run(now, speed, stride, cycles, pl, yield)) return false;
        pl->speed = speed;
        if (pl->peak50 <= MAX_JOINT_SPEED_DEG_S) return true;
        /* joint speed is about proportional to the gait speed */
        speed *= 0.98f * MAX_JOINT_SPEED_DEG_S / pl->peak50;
        pl->slowed = true;
        if (speed < SPEED_MIN) break;
    }
    pl->ok = false;
    pl->why = "cannot keep the joints under MAX_JOINT_SPEED_DEG_S";
    pl->why_servo = -1;
    return false;
}
