/* Joint angles q <-> servo degrees, and the windows every target is checked
 * against. No hardware access: also built on the host (tools/plan_host.c). */
#ifndef GAIT_RUNNER_POSE_H
#define GAIT_RUNNER_POSE_H

#include <stdbool.h>
#include "gait.h"
#include "robot.h"

bool  pose_calibrated(int n);       /* CFG_CAL_POSE set */
float pose_mid(int n);              /* servo deg at q = 0 */
float pose_deg(int n, float q);     /* servo deg of q */
float pose_q(int n, float deg);     /* q of a servo deg */
float pose_cal_cmd(int n);          /* servo deg at the calibrator pose */

/* NULL if q is a target servo n may be sent, else why not. *deg gets its servo
 * deg. Checks: Q_WINDOW, TIBIA_Q_SAFE, the pulse range, the measured limits
 * (the last two only for a calibrated servo). */
const char *pose_check(int n, float q, float *deg);

/* Start-up check of a measured position: the window widened by START_MARGIN_DEG,
 * the tibia linkage window and the pulse range. */
const char *pose_check_start(int n, float q, float deg);

/* q of every servo with each foot at height H and offset D1 under its hip, body
 * joints at 0. False if out of reach. */
bool  pose_legs(float H, float D1, float q[SERVO_COUNT]);

/* q of every servo for the gait targets (body joints at 0). */
void  pose_of_gait(const gait_t *g, float q[SERVO_COUNT]);

#endif
