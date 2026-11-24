#include "guard.h"
#include "config.h"
#include "console.h"
#include "robot.h"

static const float HAND[SERVO_COUNT][2] = CFG_HAND_RANGE;
static const float REF_POT[SERVO_COUNT] = CFG_REF_POSE;
static const float Q_AT_REF[NJ]         = REF_Q;
static const float TIBIA_SAFE[2]        = TIBIA_Q_SAFE;

static bool is_set(float v) { return v > UNSET + 1.0f; }

bool  guard_has_ref(int n)  { return is_set(REF_POT[n]); }
bool  guard_has_hand(int n) { return is_set(HAND[n][0]) && is_set(HAND[n][1]) && HAND[n][1] > HAND[n][0]; }
float guard_hand(int n, int hi) { return HAND[n][hi ? 1 : 0]; }
float guard_ref(int n)      { return REF_POT[n]; }

/* servo deg = mid + dir * q, and at the calibrator q = Q_AT_REF, pot = REF_POT. */
float guard_q_of_pot(int n, float pot_deg)
{
    return Q_AT_REF[SERVO[n].joint] + (pot_deg - REF_POT[n]) * (float)SERVO[n].dir;
}

float guard_pot_of_q(int n, float q)
{
    return REF_POT[n] + (float)SERVO[n].dir * (q - Q_AT_REF[SERVO[n].joint]);
}

static void tighten(window_t *w, float lo, float hi, const char *src)
{
    if (lo > w->lo) { w->lo = lo; w->lo_src = src; }
    if (hi < w->hi) { w->hi = hi; w->hi_src = src; }
}

window_t guard_window(int n, int j, float start, float travel, float offset)
{
    window_t w = { ABS_MIN_DEG, ABS_MAX_DEG, "pulse range", "pulse range" };
    tighten(&w, start - travel, start + travel, "travel cap");
    if (guard_has_hand(n))
        tighten(&w, HAND[n][0] - offset - HAND_MARGIN_DEG, HAND[n][1] - offset + HAND_MARGIN_DEG, "hand range");
    if (j == JOINT_TIBIA && guard_has_ref(n)) {
        float a = guard_pot_of_q(n, TIBIA_SAFE[0]) - offset;
        float b = guard_pot_of_q(n, TIBIA_SAFE[1]) - offset;
        tighten(&w, a < b ? a : b, a < b ? b : a, "tibia linkage model");
    }
    return w;
}

void guard_print(int n, const window_t *w, float offset)
{
    con_printf("  %s window %.1f .. %.1f deg (%s .. %s)", SERVO[n].name, w->lo, w->hi, w->lo_src, w->hi_src);
    if (guard_has_ref(n))
        con_printf(", q %.1f .. %.1f", guard_q_of_pot(n, w->lo + offset), guard_q_of_pot(n, w->hi + offset));
    con_puts("\r\n");
}
