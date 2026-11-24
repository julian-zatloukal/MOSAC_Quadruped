/* PROGRAM_CHECK: one servo of the leg at a time, the others limp.
 *   1. pot reading valid and trusted, pulses off
 *   2. switch-on where it is (1 ms probe, settle, +-2 deg wiggle)
 *   3. for each amplitude A: +A, back, -A, back, at 5 deg/s, holding 0.4 s at each
 *      -> pot offset, gain (pot change / commanded change), backlash (the same
 *         target approached from above and from below), tracking error
 *   4. back to where it started, pulses off (no drop: that is its rest pose)
 * No neutral angle and no joint limit is needed: it never moves more than A away
 * from where the joint already was, and never out of its window (hand range,
 * linkage model). */
#include "config.h"
#include "console.h"
#include "feedback.h"
#include "guard.h"
#include "motion.h"
#include "programs.h"
#include "robot.h"
#include <string.h>

static const float AMP[] = CHECK_AMPLITUDES;
#define NAMP ((int)(sizeof AMP / sizeof AMP[0]))

typedef enum { CK_NOT_RUN, CK_PASS, CK_WARN, CK_REFUSED, CK_SWITCH_ON, CK_BLOCKED, CK_STOPPED, CK_CUT } ck_status_t;
static const char *const CK_TEXT[] = {
    "not run", "PASS", "CHECK", "REFUSED", "FAILED AT SWITCH-ON", "BLOCKED", "stopped", "CUT",
};

typedef struct {
    ck_status_t st;
    float start, offset, track;
    float gain[NAMP], backlash[NAMP];
    bool  amp_done[NAMP];
} ck_t;

static ck_status_t status_of(result_t r)
{
    switch (r) {
    case R_OK:      return CK_PASS;
    case R_STOPPED: return CK_STOPPED;
    case R_BLOCKED: return CK_BLOCKED;
    case R_REFUSED: return CK_REFUSED;
    default:        return CK_CUT;
    }
}

static result_t go(int j, float target, float *pot)
{
    result_t r = mo_move_joint(j, target, CHECK_SPEED_DEG_S, CHECK_HOLD_MS);
    *pot = fb_pot[mo_j[j].servo].deg;
    if (r == R_OK)
        con_printf("    sent %.1f  pot %.1f  error %+.1f\r\n", mo_j[j].out, *pot, mo_err(j));
    return r;
}

static void check_one(int j, ck_t *c)
{
    int n = mo_j[j].servo;
    float amax = 0.0f;
    for (int a = 0; a < NAMP; a++) if (AMP[a] > amax) amax = AMP[a];

    con_printf("--- %s (%s, servo %d, %s, PWM %s, pot %s, model %s) ---\r\n", SERVO[n].name, JOINT_NAME[j], n,
               SERVO[n].label, SERVO[n].pwm_pin, SERVO[n].fb_pin, fb_model_name(n));
    if (mo_hold(300) != R_OK) { c->st = CK_STOPPED; return; }
    con_printf("  pulses off: pot %.1f deg (adc %u, spread %u)\r\n", fb_pot[n].deg, (unsigned)fb_pot[n].raw,
               (unsigned)fb_pot[n].iqr);

    result_t r = mo_switch_on(j);
    if (r != R_OK) { c->st = r == R_REFUSED ? CK_REFUSED : (r == R_STOPPED ? CK_STOPPED : CK_SWITCH_ON); return; }
    c->start  = mo_j[j].out;
    c->offset = mo_j[j].offset;
    mo_track_max[j] = 0.0f;

    window_t w = guard_window(n, j, c->start, amax + 1.0f, c->offset);
    mo_set_window(j, w.lo, w.hi);
    guard_print(n, &w, c->offset);

    for (int a = 0; a < NAMP && r == R_OK; a++) {
        float up = AMP[a], dn = AMP[a];
        if (up > mo_j[j].hi - c->start) up = mo_j[j].hi - c->start;
        if (dn > c->start - mo_j[j].lo) dn = c->start - mo_j[j].lo;
        if (up < 1.0f || dn < 1.0f) {
            con_printf("  +-%.0f deg: no room inside the window, skipped\r\n", AMP[a]);
            continue;
        }
        float p_hi, p_mid_above, p_lo, p_mid_below;
        con_printf("  sweep +%.1f / -%.1f deg:\r\n", up, dn);
        if ((r = go(j, c->start + up, &p_hi))        != R_OK) break;
        if ((r = go(j, c->start,      &p_mid_above)) != R_OK) break;
        if ((r = go(j, c->start - dn, &p_lo))        != R_OK) break;
        if ((r = go(j, c->start,      &p_mid_below)) != R_OK) break;
        c->gain[a]     = (p_hi - p_lo) / (up + dn);
        c->backlash[a] = p_mid_above - p_mid_below;
        c->amp_done[a] = true;
        con_printf("  -> gain %.2f, backlash %+.1f deg\r\n", c->gain[a], c->backlash[a]);
    }
    c->track = mo_track_max[j];

    result_t rp = mo_park_joint(j);
    if (r == R_OK) r = rp;
    if (r != R_OK) { c->st = status_of(r); return; }

    int last = -1;
    for (int a = 0; a < NAMP; a++) if (c->amp_done[a]) last = a;
    bool ok = last >= 0 && c->gain[last] >= CHECK_GAIN_MIN && c->gain[last] <= CHECK_GAIN_MAX &&
              c->backlash[last] < CHECK_BACKLASH_MAX && c->backlash[last] > -CHECK_BACKLASH_MAX &&
              c->track < MOVE_BLOCK_DEG;
    c->st = ok ? CK_PASS : CK_WARN;
    con_printf("  pulses off, result %s\r\n", CK_TEXT[c->st]);
}

void prog_check(void)
{
    static ck_t res[NJ];
    memset(res, 0, sizeof res);
    con_printf("=== servo check, %s leg: one servo at a time, +-A deg around where it is, %.0f deg/s ===\r\n",
               LEG[TEST_LEG].name, CHECK_SPEED_DEG_S);

    for (int j = 0; j < NJ; j++) {
        if (!(CHECK_JOINTS & (1u << j))) continue;
        check_one(j, &res[j]);
        if (res[j].st == CK_STOPPED || res[j].st == CK_CUT || mo_was_cut()) break;
        if (mo_hold(500) != R_OK) break;
    }

    con_puts("=== servo check summary ===\r\n");
    bool all_pass = true;
    for (int j = 0; j < NJ; j++) {
        if (!(CHECK_JOINTS & (1u << j))) continue;
        const ck_t *c = &res[j];
        int n = mo_j[j].servo;
        con_printf("  %s  %s", SERVO[n].name, CK_TEXT[c->st]);
        if (c->st == CK_PASS || c->st == CK_WARN || c->st == CK_BLOCKED) {
            con_printf("  start %.1f  pot offset %+.1f  gain", c->start, c->offset);
            for (int a = 0; a < NAMP; a++) con_printf(a ? "/%.2f" : " %.2f", c->amp_done[a] ? c->gain[a] : 0.0f);
            con_puts("  backlash");
            for (int a = 0; a < NAMP; a++) con_printf(a ? "/%+.1f" : " %+.1f", c->amp_done[a] ? c->backlash[a] : 0.0f);
            con_printf("  tracking max %.1f", c->track);
        }
        con_puts("\r\n");
        int last = NAMP - 1;
        con_printf("RES,check,%s,%s,%s,start,%.2f,offset,%.2f,gain,%.3f,backlash,%.2f,track,%.2f\r\n",
                   LEG[TEST_LEG].tag, SERVO[n].name, CK_TEXT[c->st], c->start, c->offset,
                   c->amp_done[last] ? c->gain[last] : 0.0f, c->amp_done[last] ? c->backlash[last] : 0.0f, c->track);
        if (c->st != CK_PASS) all_pass = false;
    }
    mo_note_stop(all_pass ? "servo check finished: all PASS" : "servo check finished: see the summary", -1);
}
