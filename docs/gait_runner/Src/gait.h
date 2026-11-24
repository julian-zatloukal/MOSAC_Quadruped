/* The walk of Programmed_Yaw (Webots/controllers/Programmed_Yaw) in C: the gait
 * generator of Programmed_Gait_V2/gait_core.py (GaitGenerator, servo_angles) and
 * the stride ramp and stop of motion_core.py (WalkController), walking forward.
 *
 * Left out, because the real robot cannot use them yet: the heading loop (it needs
 * a yaw reading; the IMU sketch does not send one), turning (set_twist with a
 * turn), walking backwards and turning on the spot. Without them the shoulder
 * (body) joints stay at q = 0, as in gait_core when no turn is commanded.
 *
 * tools/test_gait.py checks this file against the Python code step by step.
 * No hardware access here: the same file builds for the board and the host. */
#ifndef GAIT_RUNNER_GAIT_H
#define GAIT_RUNNER_GAIT_H

#include <stdbool.h>

#define GAIT_STAGES 16

/* Legs in gait_core order. */
enum { G_BL, G_FL, G_BR, G_FR, G_LEGS };

typedef struct {
    double period;              /* s, one 16-stage cycle in Webots                */
    double weight[4];           /* share of the cycle: lean, swing, restore, push */
    double ramp_cycles;         /* cycles the stride takes to grow to full        */
    float  standing_height;     /* mm, hip to foot                                */
    float  parabola_height;     /* mm, foot lift                                  */
    float  parabola_length;     /* mm, stride                                     */
    float  lean_primary;        /* mm                                             */
    float  lean_secondary;      /* mm                                             */
} gait_params_t;

typedef struct {
    gait_params_t p;
    double edge[GAIT_STAGES], width[GAIT_STAGES];
    double t;                   /* gait time, s                                   */
    int    stage;               /* stage of the last step, -1 before the first    */
    float  H[G_LEGS], D1[G_LEGS];
    float  stride_scale[G_LEGS];
    float  swing_start[G_LEGS], landing[G_LEGS];
    float  femur[G_LEGS], tibia[G_LEGS];    /* joint targets q, rad (Webots)   */
    bool   ik_ok;               /* false: a foot went out of reach; the targets
                                   are those of the last reachable step         */
    /* WalkController */
    bool   walking;             /* mode "walk" (else "stop")                      */
    bool   idle;                /* stopped and centred: the gait is not stepped   */
    double forward, wanted;     /* stride now / asked for, fraction of the full   */
    double stop_t;
} gait_t;

/* Standing pose (gait_core stand(): every foot under its hip), stopped and idle. */
void gait_init(gait_t *g, const gait_params_t *p);

/* WalkController.drive(forward) / stop(). */
void gait_drive(gait_t *g, double forward);
void gait_stop(gait_t *g);

/* One control step, as the loop of Programmed_Yaw.py: steps the generator unless
 * idle, then the stride ramp, or after a stop the check that every foot is
 * centred. dt is gait time (the board passes TICK * speed scale). */
void gait_tick(gait_t *g, double dt);

double gait_phase(const gait_t *g);     /* 0..1 in the cycle */

/* gait_core.servo_angles for one leg: hip-to-foot height H and foot offset D1
 * (mm, positive = further back) -> femur and tibia q (rad). False if out of reach. */
bool gait_leg_ik(float H, float D1, float *femur, float *tibia);

#endif
