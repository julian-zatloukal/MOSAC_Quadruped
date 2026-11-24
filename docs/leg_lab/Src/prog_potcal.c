/* PROGRAM_POTCAL: measures the pot of one servo against its own pulse, with the
 * horn (or the link) OFF the shaft, so it can turn over its whole range without
 * load and without anything to hit. This is the only program that drives a
 * servo whose pot is untrusted: the reading is what is being measured.
 *
 *   1. first pulse at the angle the pot suggests (a jump does not matter: no load)
 *   2. up through the 20 table angles (0, 14, ... 266 deg), then down again, at
 *      10 deg/s, holding 0.8 s and reading 5 medians at each angle
 *   3. stall watch: while the command moves, the pot must move too (at least 30 %
 *      of what a DS3235 pot moves); otherwise the horn is still on and blocked,
 *      or the feedback wire is not on this servo -> cut
 *   4. straight-line fit, residual, hysteresis, comparison with the current table,
 *      and a ready-to-paste table for calibration_ADC.h (same format: sums of 5)
 *   5. optionally hold POTCAL_MOUNT_DEG until USER, to push the link on there */
#include "config.h"
#include "console.h"
#include "feedback.h"
#include "hw.h"
#include "motion.h"
#include "programs.h"
#include "robot.h"

#define MAXPTS 20

static float absf(float v) { return v < 0.0f ? -v : v; }

/* A servo pushing against something draws current, which shows as a shift of the
 * reference pots: with the horn off that must not happen. */
static float    rs0;
static uint32_t stall_ms;
static bool stall_current(int n)
{
    stall_ms = fb_ref_shift() - rs0 > SEARCH_REF_DEG ? stall_ms + TICK_MS : 0;
    if (stall_ms < 50u) return false;
    con_puts("the servo draws stall current (reference pots shifted): is the horn still on? -> cut\r\n");
    mo_cut_all("potcal: stall current, horn still on?", n);
    return true;
}

/* Moves the command linearly to `target` while checking that the pot follows. */
static result_t potcal_move(int j, float target)
{
    joint_t *J = &mo_j[j];
    int      n = J->servo;
    float    win_cmd = J->out, win_raw = (float)fb_pot[n].raw;
    uint32_t win_ms = 0, bad = 0, backwards = 0;

    for (;;) {
        mo_tick_t st = mo_tick();
        if (st == MO_CUT)  return R_CUT;
        if (st == MO_STOP) { J->cmd = J->out; return R_STOPPED; }
        if (stall_current(n)) return R_CUT;
        if (st == MO_OK) {
            float d = target - J->cmd, step = POTCAL_SPEED_DEG_S * (float)TICK_MS / 1000.0f;
            J->cmd += d > step ? step : (d < -step ? -step : d);
        }
        win_ms += TICK_MS;
        if (win_ms >= 100u) {
            float dc = J->out - win_cmd;
            float dr = (float)fb_pot[n].raw - win_raw;
            float expect = absf(dc) * NOMINAL_ADC_DEG;
            if (expect > 8.0f) {
                bad       = absf(dr) < POTCAL_MIN_FOLLOW * expect ? bad + 1u : 0u;
                backwards = dr * dc < 0.0f && absf(dr) > POTCAL_MIN_FOLLOW * expect ? backwards + 1u : 0u;
            }
            if (bad >= 3u) {
                con_printf("pot not following: command moved %.1f deg, pot %.0f codes (expected about %.0f)\r\n",
                           dc, dr, expect);
                con_puts("  -> the horn is still on and the joint is blocked, or the feedback wire is not on this\r\n"
                         "     servo's pot (or the pot has no supply).\r\n");
                mo_cut_all("potcal: pot not following the servo", n);
                return R_CUT;
            }
            if (backwards >= 3u) {
                con_puts("pot reads backwards (code falls while the angle rises): wiper wired to the other end?\r\n");
                mo_cut_all("potcal: pot reads backwards", n);
                return R_CUT;
            }
            win_ms = 0;
            win_cmd = J->out;
            win_raw = (float)fb_pot[n].raw;
        }
        if (absf(J->out - target) < 0.01f) return R_OK;
    }
}

/* Sum of 5 readings 60 ms apart, like the MEAS_PER_ANG sums of calibration_ADC.h. */
static result_t potcal_read(int n, float *sum)
{
    *sum = 0.0f;
    for (int i = 0; i < fb_meas_per_angle(); i++) {
        result_t r = mo_hold(60);
        if (r != R_OK) return r;
        *sum += fb_pot[n].code;
    }
    return R_OK;
}

void prog_potcal(void)
{
    const int j = POTCAL_JOINT;
    joint_t  *J = &mo_j[j];
    const int n = J->servo;
    const int K = fb_table_points();
    const int M = fb_meas_per_angle();
    static float up[MAXPTS], down[MAXPTS];

    con_printf("=== pot calibration: %s (%s %s, servo %d, PWM %s, pot %s) ===\r\n", SERVO[n].name,
               LEG[TEST_LEG].name, JOINT_NAME[j], n, SERVO[n].pwm_pin, SERVO[n].fb_pin);
#if !POTCAL_HORN_REMOVED
    con_puts("Refused: POTCAL_HORN_REMOVED is 0 in config.h.\r\n"
             "This program turns the servo over its WHOLE range (0..266 deg). Take the horn or the link\r\n"
             "off the shaft first, so nothing can be hit, then set POTCAL_HORN_REMOVED 1 and rebuild.\r\n");
    mo_note_stop("potcal refused: horn not confirmed removed", n);
    return;
#endif
    if (K > MAXPTS) { con_puts("calibration table too long\r\n"); return; }

    uint16_t raw = fb_pot[n].raw;
    if (raw < 50u || raw > 4045u) {
        con_printf("Refused: pot reads %u, at an ADC rail: the feedback wire is not connected (or shorted).\r\n",
                   (unsigned)raw);
        mo_note_stop("potcal refused: pot at an ADC rail", n);
        return;
    }
    float est = fb_trusted(n) && fb_pot[n].valid ? fb_pot[n].deg : ((float)raw - NOMINAL_ADC0) / NOMINAL_ADC_DEG;
    if (est < 0.0f) est = 0.0f;
    if (est > 266.0f) est = 266.0f;
    con_printf("pot reads %u (%.1f deg by the %s): first pulse there. Horn off, so a jump does no harm.\r\n",
               (unsigned)raw, est, fb_trusted(n) ? "table" : "nominal line");

    /* This program owns the joint: full range, no pot-based checks (the pot is
     * what is measured); the stall watch above and the electrical watch stay on. */
    J->lo = 0.0f;
    J->hi = 266.0f;
    J->cmd = J->out = J->rest = est;
    J->offset = 0.0f;
    J->offset_known = false;
    J->exempt = true;
    rs0 = fb_ref_shift();
    stall_ms = 0;
    J->on = true;
    result_t r = R_OK;
    for (int i = 0; i < 50 && r == R_OK; i++) {          /* 0.5 s at the first pulse */
        mo_tick_t st = mo_tick();
        if (st == MO_CUT) r = R_CUT;
        else if (st == MO_STOP) r = R_STOPPED;
        else if (stall_current(n)) r = R_CUT;
    }

    for (int k = 0; k < K && r == R_OK; k++) {
        if ((r = potcal_move(j, fb_angle(k))) != R_OK) break;
        if ((r = mo_hold(POTCAL_HOLD_MS)) != R_OK) break;
        if ((r = potcal_read(n, &up[k])) != R_OK) break;
        con_printf("  up   %5.1f deg (%u us): adc %.1f\r\n", fb_angle(k), (unsigned)mo_us(fb_angle(k)), up[k] / M);
    }
    for (int k = K - 1; k >= 0 && r == R_OK; k--) {
        if ((r = potcal_move(j, fb_angle(k))) != R_OK) break;
        if ((r = mo_hold(POTCAL_HOLD_MS)) != R_OK) break;
        if ((r = potcal_read(n, &down[k])) != R_OK) break;
        con_printf("  down %5.1f deg (%u us): adc %.1f\r\n", fb_angle(k), (unsigned)mo_us(fb_angle(k)), down[k] / M);
    }
    if (r != R_OK) {
        if (J->on) mo_cut(j);           /* horn off: going limp is harmless */
        con_puts("pot calibration interrupted, pulses off\r\n");
        return;
    }

    /* Least squares over both sweeps: code = a + b * deg. */
    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    int   cnt = 0;
    for (int k = 0; k < K; k++) {
        float x = fb_angle(k);
        float ys[2] = { up[k] / M, down[k] / M };
        for (int s = 0; s < 2; s++) { sx += x; sy += ys[s]; sxx += x * x; sxy += x * ys[s]; cnt++; }
    }
    float b = (cnt * sxy - sx * sy) / (cnt * sxx - sx * sx);
    float a = (sy - b * sx) / cnt;
    float resid = 0.0f, hyst = 0.0f, tab_sum = 0.0f, tab_min = 1e9f, tab_max = -1e9f;
    bool  mono = true;
    for (int k = 0; k < K; k++) {
        float x = fb_angle(k);
        float e1 = absf(up[k] / M - (a + b * x)) / b, e2 = absf(down[k] / M - (a + b * x)) / b;
        if (e1 > resid) resid = e1;
        if (e2 > resid) resid = e2;
        float h = absf(up[k] - down[k]) / M / b;
        if (h > hyst) hyst = h;
        if (k && (up[k] <= up[k - 1] || down[k] <= down[k - 1])) mono = false;
        float td = fb_table_deg(n, (up[k] + down[k]) * 0.5f / M) - x;   /* what the stored table would read */
        float mc = (up[k] + down[k]) * 0.5f / M;
        if (mc > fb_table_code(n, 0, 1) && mc < fb_table_code(n, K - 1, 1)) {   /* the table clips outside */
            tab_sum += td;
            if (td < tab_min) tab_min = td;
            if (td > tab_max) tab_max = td;
        }
    }
    int tab_n = 0;
    for (int k = 0; k < K; k++) {
        float mc = (up[k] + down[k]) * 0.5f / M;
        if (mc > fb_table_code(n, 0, 1) && mc < fb_table_code(n, K - 1, 1)) tab_n++;
    }
    float tab_mean = tab_n ? tab_sum / (float)tab_n : 0.0f;

    con_puts("=== pot calibration result ===\r\n");
    con_printf("  line: adc = %.1f + %.3f * deg   (the other servos: about 190 + 12.55..12.70 * deg)\r\n", a, b);
    con_printf("  largest deviation from the line %.2f deg, largest up/down difference %.2f deg%s\r\n", resid, hyst,
               mono ? "" : ", NOT MONOTONIC");
    con_printf("  the current table for this position (%s) reads it %+.1f deg off on average (%+.1f .. %+.1f)\r\n",
               SERVO[n].label, tab_mean, tab_min, tab_max);
    if (!mono || resid > 2.0f)
        con_puts("  VERDICT: not usable as it is: the reading is not a clean line (bad wiper contact, noise,\r\n"
                 "           or a wire on the wrong pot). Fix the wiring and run this again.\r\n");
    else if (b < 11.0f || b > 14.5f)
        con_puts("  VERDICT: usable, but a different pot or supply from the other servos: use this table/line.\r\n");
    else
        con_puts("  VERDICT: usable: same kind of pot as the other servos; use this table/line for it.\r\n");
    con_printf("RES,potfit,%s,a,%.2f,b,%.4f,resid,%.2f,hyst,%.2f,table_offset,%.2f,monotonic,%d\r\n",
               SERVO[n].name, a, b, resid, hyst, tab_mean, mono ? 1 : 0);
    for (int k = 0; k < K; k++)
        con_printf("RES,potcal,%s,%.0f,%.1f,%.1f\r\n", SERVO[n].name, fb_angle(k), up[k] / M, down[k] / M);

    con_printf("\r\n/* calibration_ADC.h: replace ADC_VALUES_SERVO_%d (measured by leg_lab PROGRAM_POTCAL) */\r\n", n);
    con_printf("const float ADC_VALUES_SERVO_%d[2][MED_CALIB] = // %s - new servo, measured with the horn off\r\n{\r\n", n,
               SERVO[n].name);
    con_puts("\t// ADC accumulated values for 5 measurements of servo going down\r\n\t{\r\n");
    for (int k = 0; k < K; k++) con_printf("\t\t%.0f%s\r\n", down[k], k < K - 1 ? "," : "");
    con_puts("\t},\r\n\t// ADC accumulated values for 5 measurements of servo going up\r\n\t{\r\n");
    for (int k = 0; k < K; k++) con_printf("\t\t%.0f%s\r\n", up[k], k < K - 1 ? "," : "");
    con_puts("\t}\r\n};\r\n");
    con_printf("/* or in leg_lab config.h: CFG_POT_MODEL entry %d = POT_LINE, CFG_POT_LINE entry %d = { %.1ff, %.4ff } */\r\n\r\n",
               n, n, a, b);

    if (POTCAL_MOUNT_DEG > UNSET + 1.0f) {
        if ((r = potcal_move(j, POTCAL_MOUNT_DEG)) == R_OK) {
            con_printf("holding %.1f deg (%u us). Push the link on at the tooth nearest the calibrator pose,\r\n"
                       "then press USER (or send 'x'): the pulses stop and the servo goes limp.\r\n",
                       POTCAL_MOUNT_DEG, (unsigned)mo_us(POTCAL_MOUNT_DEG));
            for (;;) {
                mo_tick_t st = mo_tick();
                if (st == MO_STOP || st == MO_CUT) break;
            }
        }
    }
    if (J->on) mo_cut(j);
    mo_note_stop("pot calibration done", n);
    con_puts("pot calibration done, pulses off\r\n");
}
