#include "gait.h"
#include <math.h>

/* Leg geometry from the CAD, as gait_core.py (mm, rad). */
#define PI_F       3.14159265358979f
#define L_FEMUR    100.0f
#define L_TIBIA    100.70866f
#define PSI        (2.51591f * PI_F / 180.0f)
#define LK_A       24.0f            /* tibia servo -> tibia four-bar linkage */
#define LK_B       40.5f
#define LK_C       28.66176f
#define LK_D       27.0f
#define LK_DELTA   0.7984834029f
#define LK_EPSILON 1.570796327f
#define MID_SOLID_FEMUR (42.65021f * PI_F / 180.0f)
#define MID_SOLID_TIBIA (16.75397157f * PI_F / 180.0f)

enum { K_LEAN, K_SWING, K_RESTORE, K_PUSH };

/* gait_core.STAGES: (kind, primary leg, secondary leg). */
static const struct { unsigned char kind, leg, other; } STAGE[GAIT_STAGES] = {
    { K_LEAN, G_BR, G_BL }, { K_SWING, G_FL, 0 }, { K_RESTORE, G_BR, G_BL }, { K_PUSH, 0, 0 },
    { K_LEAN, G_FL, G_FR }, { K_SWING, G_BR, 0 }, { K_RESTORE, G_FL, G_FR }, { K_PUSH, 0, 0 },
    { K_LEAN, G_BL, G_BR }, { K_SWING, G_FR, 0 }, { K_RESTORE, G_BL, G_BR }, { K_PUSH, 0, 0 },
    { K_LEAN, G_FR, G_FL }, { K_SWING, G_BL, 0 }, { K_RESTORE, G_FR, G_FL }, { K_PUSH, 0, 0 },
};

bool gait_leg_ik(float H, float D1, float *femur, float *tibia)
{
    if (!(H > 1.0f)) return false;
    float d2s = H * H + D1 * D1;
    float d2  = sqrtf(d2s);
    float a   = atanf(D1 / H);
    float cf  = (d2s + L_FEMUR * L_FEMUR - L_TIBIA * L_TIBIA) / (2.0f * L_FEMUR * d2);
    float ct  = (d2s + L_TIBIA * L_TIBIA - L_FEMUR * L_FEMUR) / (2.0f * L_TIBIA * d2);
    if (!(cf >= -1.0f && cf <= 1.0f && ct >= -1.0f && ct <= 1.0f)) return false;
    float phi_femur = PI_F / 2.0f - a - acosf(cf);
    float phi_tibia = PI_F / 2.0f + a - acosf(ct) + PSI;

    /* tibia angle -> tibia servo angle through the four-bar linkage */
    float ws = LK_C * LK_C + LK_D * LK_D - 2.0f * LK_C * LK_D * cosf(PI_F + LK_DELTA - LK_EPSILON - phi_tibia);
    if (!(ws > 0.0f)) return false;
    float w  = sqrtf(ws);
    float c1 = (LK_C * LK_C + ws - LK_D * LK_D) / (2.0f * LK_C * w);
    float c2 = (LK_A * LK_A + ws - LK_B * LK_B) / (2.0f * LK_A * w);
    if (!(c1 >= -1.0f && c1 <= 1.0f && c2 >= -1.0f && c2 <= 1.0f)) return false;
    float phi_tibia_servo = PI_F - LK_DELTA - acosf(c1) - acosf(c2);

    /* kinematics angles -> Webots joint targets (servo_angles: -(-phi + mid)) */
    *femur = phi_femur - MID_SOLID_FEMUR;
    *tibia = phi_tibia_servo - MID_SOLID_TIBIA;
    return true;
}

static void solve(gait_t *g)
{
    float f[G_LEGS], t[G_LEGS];
    for (int k = 0; k < G_LEGS; k++) {
        if (!gait_leg_ik(g->H[k], g->D1[k], &f[k], &t[k])) { g->ik_ok = false; return; }
    }
    for (int k = 0; k < G_LEGS; k++) { g->femur[k] = f[k]; g->tibia[k] = t[k]; }
    g->ik_ok = true;
}

static void timing(gait_t *g)
{
    double w[GAIT_STAGES], sum = 0.0, acc = 0.0;
    for (int i = 0; i < GAIT_STAGES; i++) { w[i] = g->p.weight[STAGE[i].kind]; sum += w[i]; }
    for (int i = 0; i < GAIT_STAGES; i++) {
        g->width[i] = w[i] / sum;
        g->edge[i]  = acc / sum;
        acc += w[i];
    }
}

double gait_phase(const gait_t *g) { return fmod(g->t, g->p.period) / g->p.period; }

/* (stage, progress 0..1 in it) at this fraction of the cycle: _locate(). */
static int locate(const gait_t *g, double phase, double *prog)
{
    int s = 0;
    for (int i = 1; i < GAIT_STAGES; i++) if (g->edge[i] <= phase) s = i;
    *prog = (phase - g->edge[s]) / g->width[s];
    return s;
}

static void set_stride(gait_t *g, double forward)
{
    for (int k = 0; k < G_LEGS; k++) g->stride_scale[k] = (float)forward;
}

void gait_init(gait_t *g, const gait_params_t *p)
{
    g->p = *p;
    timing(g);
    g->t = 0.0;
    g->stage = -1;
    for (int k = 0; k < G_LEGS; k++) {
        g->H[k] = p->standing_height;
        g->D1[k] = 0.0f;
        g->swing_start[k] = g->landing[k] = 0.0f;
    }
    set_stride(g, 0.0);
    g->walking = false;
    g->idle = true;
    g->forward = g->wanted = 0.0;
    g->stop_t = 0.0;
    solve(g);
}

void gait_drive(gait_t *g, double forward)
{
    if (forward <= 0.0) { gait_stop(g); return; }
    if (!g->walking) { g->walking = true; g->idle = false; }
    g->wanted = forward;
}

void gait_stop(gait_t *g)
{
    if (g->walking) { g->walking = false; g->stop_t = g->t; }
    g->forward = g->wanted = 0.0;
    set_stride(g, 0.0);
}

/* GaitGenerator.step() with finish_stages on (motion_core forces it). */
static void step(gait_t *g, double dt)
{
    const gait_params_t *p = &g->p;
    double prog;
    int    s = locate(g, gait_phase(g), &prog);
    double stage_time = p->period * g->width[s];
    prog += dt / stage_time;
    if (prog > 1.0) prog = 1.0;

    int kind = STAGE[s].kind, leg = STAGE[s].leg, other = STAGE[s].other;
    if (s != g->stage) {
        g->stage = s;
        if (kind == K_SWING) {      /* the swing starts from where the foot is */
            g->swing_start[leg] = g->D1[leg];
            g->landing[leg]     = -g->stride_scale[leg] * p->parabola_length / 2.0f;
        }
    }

    float pr = (float)prog, standing = p->standing_height;
    switch (kind) {
    case K_LEAN:
        g->H[leg]   = standing - p->lean_primary * pr;
        g->H[other] = standing - p->lean_secondary * pr;
        break;
    case K_RESTORE:
        g->H[leg]   = standing - p->lean_primary * (1.0f - pr);
        g->H[other] = standing - p->lean_secondary * (1.0f - pr);
        break;
    case K_SWING: {
        float c = 0.5f - pr;
        g->H[leg]  = standing - p->parabola_height * (1.0f - 4.0f * c * c);
        g->D1[leg] = g->swing_start[leg] + (g->landing[leg] - g->swing_start[leg]) * pr;
        break;
    }
    default: {                      /* push: every foot slides back a quarter stride */
        float share = (float)(dt / stage_time / 4.0);
        for (int k = 0; k < G_LEGS; k++) g->D1[k] += g->stride_scale[k] * p->parabola_length * share;
        break;
    }
    }
    g->t += dt;
    solve(g);
}

static bool is_push(int stage) { return stage >= 0 && STAGE[stage].kind == K_PUSH; }

void gait_tick(gait_t *g, double dt)
{
    int prev = g->stage;
    if (!g->idle) step(g, dt);
    bool push_just_finished = is_push(prev) && g->stage != prev;

    if (g->walking) {               /* speeding up is ramped, slowing down is not */
        double per_step = dt / g->p.period;
        if (g->wanted > g->forward) {
            g->forward += per_step / g->p.ramp_cycles;
            if (g->forward > g->wanted) g->forward = g->wanted;
        } else {
            g->forward = g->wanted;
        }
        set_stride(g, g->forward);
    } else if (!g->idle && push_just_finished && g->t >= g->stop_t + g->p.period) {
        g->idle = true;             /* every foot has had a swing with no stride: all centred */
    }
}
