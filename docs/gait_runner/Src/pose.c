#include "pose.h"
#include "config.h"

#define RAD2DEG 57.29577951f

static const float CAL[SERVO_COUNT]          = CFG_CAL_POSE;
static const float TRIM[SERVO_COUNT]         = CFG_TRIM;
static const float LIMIT[SERVO_COUNT][2]     = CFG_CMD_LIMITS;
static const float REFQ[NJ]                  = REF_Q;
static const float QWIN[NJ][2]               = Q_WINDOW;
static const float TSAFE[2]                  = TIBIA_Q_SAFE;

static bool set(float v) { return v > UNSET + 1.0f; }

bool  pose_calibrated(int n) { return set(CAL[n]); }
float pose_mid(int n)        { return CAL[n] - SERVO[n].dir * REFQ[SERVO[n].joint] + TRIM[n]; }
float pose_deg(int n, float q)   { return pose_mid(n) + SERVO[n].dir * q; }
float pose_q(int n, float deg)   { return (deg - pose_mid(n)) * SERVO[n].dir; }
float pose_cal_cmd(int n)    { return pose_deg(n, REFQ[SERVO[n].joint]); }

static const char *limits(int n, float deg, float margin)
{
    if (!pose_calibrated(n)) return 0;
    if (deg < ABS_MIN_DEG || deg > ABS_MAX_DEG) return "outside the pulse range (ABS_MIN_DEG..ABS_MAX_DEG)";
    if (set(LIMIT[n][0]) && deg < LIMIT[n][0] + margin) return "too close to its measured low limit (CFG_CMD_LIMITS)";
    if (set(LIMIT[n][1]) && deg > LIMIT[n][1] - margin) return "too close to its measured high limit (CFG_CMD_LIMITS)";
    return 0;
}

const char *pose_check(int n, float q, float *deg)
{
    int j = SERVO[n].joint;
    float d = pose_deg(n, q);
    if (deg) *deg = d;
    if (!(q >= QWIN[j][0] && q <= QWIN[j][1])) return "outside its joint window (Q_WINDOW)";
    if (j == JOINT_TIBIA && !(q >= TSAFE[0] && q <= TSAFE[1])) return "outside the tibia linkage window (TIBIA_Q_SAFE)";
    return limits(n, d, LIMIT_MARGIN_DEG);
}

const char *pose_check_start(int n, float q, float deg)
{
    int j = SERVO[n].joint;
    if (!(q >= QWIN[j][0] - START_MARGIN_DEG && q <= QWIN[j][1] + START_MARGIN_DEG))
        return "too far outside its joint window: move it by hand towards the middle";
    if (j == JOINT_TIBIA && !(q >= TSAFE[0] && q <= TSAFE[1]))
        return "tibia linkage near a dead point: move it by hand towards the middle";
    return limits(n, deg, 0.0f);
}

bool pose_legs(float H, float D1, float q[SERVO_COUNT])
{
    float f, t;
    if (!gait_leg_ik(H, D1, &f, &t)) return false;
    for (int l = 0; l < LEG_COUNT; l++) {
        q[LEG[l].servo[JOINT_BODY]]  = 0.0f;
        q[LEG[l].servo[JOINT_FEMUR]] = f * RAD2DEG;
        q[LEG[l].servo[JOINT_TIBIA]] = t * RAD2DEG;
    }
    return true;
}

void pose_of_gait(const gait_t *g, float q[SERVO_COUNT])
{
    for (int l = 0; l < LEG_COUNT; l++) {
        int k = LEG[l].gait_leg;
        q[LEG[l].servo[JOINT_BODY]]  = 0.0f;
        q[LEG[l].servo[JOINT_FEMUR]] = g->femur[k] * RAD2DEG;
        q[LEG[l].servo[JOINT_TIBIA]] = g->tibia[k] * RAD2DEG;
    }
}
