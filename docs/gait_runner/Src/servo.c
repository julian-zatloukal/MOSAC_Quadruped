#include "servo.h"
#include "config.h"
#include "console.h"
#include "hw.h"
#include "pose.h"
#include "pots.h"
#include <string.h>

servo_t sv[SERVO_COUNT];

static const float OFFSET_HINT[SERVO_COUNT] = CFG_OFFSET_HINT;
static const float DT = (float)TICK_MS / 1000.0f;

static uint32_t next_tick, next_telemetry;
static int      char_latched;
static bool     press_latched;
static uint8_t  btn_count;
static bool     btn_was_down;
static bool     paused, faulted, cut_happened;
static uint32_t pause_ms, calm_ms;
static float    track_limit = TRACK_MOVE_DEG, slew = MOVE_SLEW_DEG_S;
static float    vdda_base = 3.3f, cm_now;
static char     state_char = 'I';
static double   state_gait_t;
static char     last_stop[112] = "none since reset";

static float absf(float v) { return v < 0.0f ? -v : v; }
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static float median(float *v, int n)
{
    for (int i = 1; i < n; i++) {
        float x = v[i];
        int k = i - 1;
        while (k >= 0 && v[k] > x) { v[k + 1] = v[k]; k--; }
        v[k + 1] = x;
    }
    return (n & 1) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

static uint16_t us(float deg)
{
    deg = clampf(deg, ABS_MIN_DEG, ABS_MAX_DEG);
    return (uint16_t)(US_AT_0_DEG + deg * US_PER_DEG + 0.5f);
}

static void note_stop(const char *why, int servo)
{
    size_t n = strlen(why);
    if (n > sizeof last_stop - 12) n = sizeof last_stop - 12;
    memcpy(last_stop, why, n);
    last_stop[n] = '\0';
    if (servo >= 0) {
        strcat(last_stop, " (");
        strcat(last_stop, SERVO[servo].name);
        strcat(last_stop, ")");
    }
}

/* --------------------------------------------------------------- basics --- */

void sv_init(void)
{
    memset(sv, 0, sizeof sv);
    next_tick = next_telemetry = hw_millis();
}

void sv_begin(void)
{
    faulted = paused = cut_happened = false;
    pause_ms = calm_ms = 0;
    press_latched = false;
    char_latched = 0;
    track_limit = TRACK_MOVE_DEG;
    slew = MOVE_SLEW_DEG_S;
    strcpy(last_stop, "running");
    next_tick = next_telemetry = hw_millis();
}

int  sv_take_char(void)  { int c = char_latched; char_latched = 0; return c; }
bool sv_take_press(void) { bool p = press_latched; press_latched = false; return p; }

void sv_set_limits(float track_deg, float slew_deg_s)
{
    /* a tighter limit starts from a clean average, so the lag of a walk that just
     * ended is not taken for a fault; a lasting error builds up again in ~0.2 s */
    if (track_deg < track_limit)
        for (int n = 0; n < SERVO_COUNT; n++) sv[n].err_avg = 0.0f;
    track_limit = track_deg;
    slew = slew_deg_s;
}
void sv_set_state(char state, double gait_t) { state_char = state; state_gait_t = gait_t; }

bool sv_any_on(void)
{
    for (int n = 0; n < SERVO_COUNT; n++) if (sv[n].on) return true;
    return false;
}

bool sv_all_on(void)
{
    for (int n = 0; n < SERVO_COUNT; n++) if (!sv[n].on) return false;
    return true;
}

bool sv_faulted(void) { return faulted; }
bool sv_paused(void)  { return paused; }
const char *sv_last_stop(void) { return last_stop; }

float sv_meas(int n) { return pot[n].deg - sv[n].offset; }
float sv_err(int n)  { return sv_meas(n) - sv[n].out; }

void sv_command(int n, float deg)
{
    if (sv[n].on && !faulted) sv[n].cmd = deg;
}

void sv_cut_all(const char *why)
{
    for (int n = 0; n < SERVO_COUNT; n++) {
        sv[n].on = false;
        sv[n].offset_known = false;
        hw_set_pulse_us(n, 0);
    }
    if (!cut_happened) {
        note_stop(why, -1);
        con_printf("CUT: %s -> all pulses off\r\n", why);
    }
    cut_happened = true;
    faulted = paused = false;
    hw_led(HW_LED_GREEN, false);
    hw_led(HW_LED_RED, true);
}

/* Command every joint where it really is (if that is close), so a blocked servo
 * stops pushing. During an electrical disturbance the readings are not trusted:
 * then the commands just stop where they are. */
void sv_freeze_all(void)
{
    for (int n = 0; n < SERVO_COUNT; n++) {
        servo_t *s = &sv[n];
        if (!s->on) continue;
        float at = s->out;
        if (s->offset_known && !paused && pot[n].valid && absf(sv_err(n)) < FREEZE_MAX_DEG) at = sv_meas(n);
        s->cmd = s->out = clampf(at, ABS_MIN_DEG, ABS_MAX_DEG);
    }
}

void sv_fault(const char *why, int servo)
{
    if (faulted || cut_happened || !sv_any_on()) return;
    sv_freeze_all();
    faulted = true;
    note_stop(why, servo);
    con_printf("FAULT: %s%s%s -> frozen, holding\r\n", why, servo >= 0 ? ": " : "", servo >= 0 ? SERVO[servo].name : "");
    hw_led(HW_LED_RED, true);
}

void sv_clear_fault(void)
{
    if (!faulted) return;
    faulted = false;
    for (int n = 0; n < SERVO_COUNT; n++) { sv[n].err_avg = 0.0f; sv[n].invalid_ms = 0; }
    hw_led(HW_LED_RED, false);
}

/* -------------------------------------------------------------- one tick --- */

static bool button_edge(void)
{
    if (hw_button_down()) { if (btn_count < 3) btn_count++; }
    else btn_count = 0;
    bool down  = btn_count >= 3;            /* 30 ms debounce */
    bool event = down && !btn_was_down;
    btn_was_down = down;
    return event;
}

static void safety(void)
{
    if (!sv_any_on()) { paused = false; pause_ms = calm_ms = 0; cm_now = 0.0f; return; }

    if (pot_vdda < VDDA_MIN_V) {
        con_printf("supply: VDDA %.2f V (the Nucleo is close to brown-out)\r\n", pot_vdda);
        sv_cut_all("supply: VDDA too low");
        return;
    }

    /* Common mode: the median error of the powered servos. Joints lag and sag in
     * both directions, so it stays near 0 unless every reading shifts together. */
    float e[SERVO_COUNT];
    int   m = 0;
    for (int n = 0; n < SERVO_COUNT; n++)
        if (sv[n].on && sv[n].offset_known && pot[n].valid) e[m++] = sv_err(n);
    cm_now = m >= 4 ? median(e, m) : 0.0f;
    float dip = vdda_base - pot_vdda;

    if (absf(cm_now) > CM_PAUSE_DEG || dip > VDDA_DIP_V) {
        if (!paused)
            con_printf("electrical disturbance: all pots shifted %+.1f deg, VDDA dip %.3f V -> pause\r\n", cm_now, dip);
        paused = true;
        calm_ms = 0;
    } else if (paused) {
        calm_ms = (absf(cm_now) < CM_CLEAR_DEG && dip < 0.5f * VDDA_DIP_V) ? calm_ms + TICK_MS : 0;
        if (calm_ms >= 300u) {
            paused = false;
            pause_ms = 0;
            con_puts("electrical disturbance cleared\r\n");
        }
    }
    if (paused) {
        pause_ms += TICK_MS;
        if (pause_ms > PAUSE_MAX_MS) sv_fault("electrical disturbance did not clear", -1);
        return;                             /* readings not trusted: no joint checks */
    }

    for (int n = 0; n < SERVO_COUNT; n++) {
        servo_t *s = &sv[n];
        if (!s->on) continue;
        if (!pot[n].valid) {
            s->invalid_ms += TICK_MS;
            if (s->invalid_ms > POT_INVALID_MS) sv_fault("pot reading invalid", n);
            continue;
        }
        s->invalid_ms = 0;
        if (!s->offset_known) continue;
        s->err_avg += (absf(sv_err(n)) - s->err_avg) * ((float)TICK_MS / TRACK_TAU_MS);
        if (s->err_avg > track_limit && !faulted) {
            con_printf("%s not following: sent %.1f, at %.1f, averaged error %.1f deg (limit %.1f)\r\n",
                       SERVO[n].name, s->out, sv_meas(n), s->err_avg, track_limit);
            sv_fault("joint not following its command", n);
        }
    }
}

static void output_update(void)
{
    const float step = slew * DT;
    for (int n = 0; n < SERVO_COUNT; n++) {
        servo_t *s = &sv[n];
        if (!s->on) { hw_set_pulse_us(n, 0); continue; }
        if (faulted) s->cmd = s->out;
        s->cmd = clampf(s->cmd, ABS_MIN_DEG, ABS_MAX_DEG);
        if (!paused) s->out += clampf(s->cmd - s->out, -step, step);
        hw_set_pulse_us(n, us(s->out));
    }
}

static void telemetry(uint32_t now)
{
    con_printf("fb %.2f %c g %.2f v %.2f cm %+.1f", (float)now / 1000.0f, state_char, (float)state_gait_t,
               pot_vdda, cm_now);
    for (int n = 0; n < SERVO_COUNT; n++) {
        if (sv[n].on && sv[n].offset_known) con_printf(" %s %.1f/%+.1f", SERVO[n].name, sv[n].out, sv_err(n));
        else if (sv[n].on)                  con_printf(" %s %.1f/?", SERVO[n].name, sv[n].out);
        else                                con_printf(" %s off", SERVO[n].name);
    }
    con_puts(paused ? " PAUSED\r\n" : (faulted ? " FAULT\r\n" : "\r\n"));
}

static tk_t tick_body(void)
{
    uint32_t now = hw_millis();
    next_tick += TICK_MS;
    if ((int32_t)(now - next_tick) > 50) next_tick = now + TICK_MS;   /* fell behind: resync */

    int c = hw_uart_getc();
    if (c > 0) char_latched = c;
    if (button_edge()) press_latched = true;
    pots_update();
    if ((c == 'k' || c == 'K') && sv_any_on()) sv_cut_all("cut by user ('k')");

    safety();
    output_update();

    if (sv_any_on() && (int32_t)(now - next_telemetry) >= 0) {
        next_telemetry = now + TELEMETRY_MS;
        telemetry(now);
    }
    if (cut_happened) return TK_CUT;
    if (faulted)      return TK_FAULT;
    if (paused)       return TK_PAUSE;
    return TK_OK;
}

tk_t sv_tick(void)
{
    hw_wait_until(next_tick);
    return tick_body();
}

void sv_poll(void)
{
    if ((int32_t)(hw_millis() - next_tick) >= 0) (void)tick_body();
}

/* ------------------------------------------------------------ planned moves --- */

static float ease(float t)   /* smoothstep: zero speed at both ends, peak 1.5x mean */
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

void sv_freeze_by_user(void)
{
    sv_freeze_all();
    con_puts("stopped by user: frozen where it is\r\n");
    note_stop("stopped by user", -1);
}

bool sv_user_stop(void)
{
    int c = sv_take_char();
    if (sv_take_press() || c == 'x' || c == 'X') {
        sv_freeze_by_user();
        return true;
    }
    return false;
}

result_t sv_hold(uint32_t ms)
{
    uint32_t t0 = hw_millis();
    while (hw_millis() - t0 < ms) {
        tk_t st = sv_tick();
        if (st == TK_CUT)   return R_CUT;
        if (st == TK_FAULT) return R_FAULT;
        if (sv_user_stop())    return R_FROZEN;
    }
    return R_OK;
}

result_t sv_move(const float to[SERVO_COUNT], float speed)
{
    float from[SERVO_COUNT], tgt[SERVO_COUNT], maxd = 0.0f;
    for (int n = 0; n < SERVO_COUNT; n++) {
        from[n] = sv[n].cmd;
        tgt[n]  = sv[n].on ? clampf(to[n], ABS_MIN_DEG, ABS_MAX_DEG) : sv[n].cmd;
        if (absf(tgt[n] - from[n]) > maxd) maxd = absf(tgt[n] - from[n]);
    }
    float dur = 0.0f;
    if (maxd > 0.05f) {
        dur = 1.5f * maxd / speed;
        if (dur < (float)MIN_MOVE_MS / 1000.0f) dur = (float)MIN_MOVE_MS / 1000.0f;
    }
    float p = dur > 0.0f ? 0.0f : 1.0f;
    for (;;) {
        tk_t st = sv_tick();
        if (st == TK_CUT)   return R_CUT;
        if (st == TK_FAULT) return R_FAULT;
        if (sv_user_stop())    return R_FROZEN;
        if (st == TK_PAUSE) continue;
        if (p >= 1.0f) return R_OK;         /* the last command went out in this tick */
        p += DT / dur;
        if (p > 1.0f) p = 1.0f;
        float k = ease(p);
        for (int n = 0; n < SERVO_COUNT; n++)
            if (sv[n].on) sv[n].cmd = from[n] + (tgt[n] - from[n]) * k;
    }
}

/* After a fault the commands hold. A joint that is still off its command (it is
 * blocked, or cannot carry its load) gets its command moved slowly towards where
 * it is, YIELD_DEG_S and at most YIELD_MAX_DEG in all, so it stops pushing; a
 * wrong reading can then move a joint only slowly and only that far. A joint
 * still more than STALL_DEG off after STALL_MS: every pulse is cut. */
result_t sv_fault_hold(void)
{
    uint32_t over[SERVO_COUNT] = { 0 };
    float    given[SERVO_COUNT] = { 0 };
    const float step_max = YIELD_DEG_S * DT;
    for (uint32_t t = 0; t < STALL_MS + 200u; t += TICK_MS) {
        if (sv_tick() == TK_CUT) return R_CUT;
        for (int n = 0; n < SERVO_COUNT; n++) {
            servo_t *s = &sv[n];
            if (!s->on || !s->offset_known || !pot[n].valid || paused) { over[n] = 0; continue; }
            float e = sv_err(n);
            if (absf(e) > YIELD_FROM_DEG && given[n] < YIELD_MAX_DEG) {
                float step = clampf(e, -step_max, step_max);
                s->out = s->cmd = clampf(s->out + step, ABS_MIN_DEG, ABS_MAX_DEG);
                given[n] += absf(step);
            }
            over[n] = absf(e) > STALL_DEG ? over[n] + TICK_MS : 0;
            if (over[n] >= STALL_MS) {
                con_printf("%s still %.1f deg from its command %.1f s after the fault (gave way %.1f deg)\r\n",
                           SERVO[n].name, e, (float)STALL_MS / 1000.0f, given[n]);
                sv_cut_all("a joint stayed stalled after a fault");
                return R_CUT;
            }
        }
    }
    for (int n = 0; n < SERVO_COUNT; n++)
        if (given[n] > 0.5f) con_printf("%s gave way %.1f deg to stop pushing\r\n", SERVO[n].name, given[n]);
    return R_FAULT;
}

/* ---------------------------------------------------------------- switch-on --- */

static result_t switch_on(int n)
{
    servo_t     *s   = &sv[n];
    const pot_t *p   = &pot[n];
    int          leg = SERVO[n].leg;

    if (!p->valid) {
        con_printf("  %s: pot reading invalid (adc %u, spread %u)\r\n", SERVO[n].name, (unsigned)p->raw, (unsigned)p->iqr);
        note_stop("switch-on: invalid pot reading", n);
        return R_REFUSED;
    }
    float first = p->deg - OFFSET_HINT[n];

    /* Probe: first pulse where the pot says the servo is, the pots of its leg
     * watched every 1 ms. The pots of the other legs (still or holding) are the
     * references: if they shift too, the shift is electrical, not motion. */
    float ref0[SERVO_COUNT], prev[SERVO_COUNT], peak = 0.0f;
    bool  watch[SERVO_COUNT], refm[SERVO_COUNT];
    for (int m = 0; m < SERVO_COUNT; m++) {
        watch[m] = SERVO[m].leg == leg && pot[m].valid;
        refm[m]  = SERVO[m].leg != leg && pot[m].iqr <= FB_MAX_IQR && pot[m].raw > 50u && pot[m].raw < 4045u;
        ref0[m]  = prev[m] = pot_fast_deg(m);
    }
    s->cmd = s->out = first;
    s->offset = OFFSET_HINT[n];
    s->offset_known = false;
    s->err_avg = 0.0f;
    s->invalid_ms = 0;
    s->on = true;
    hw_set_pulse_us(n, us(first));
    con_printf("  %s: first pulse at %.1f deg (%u us), pot %.1f\r\n", SERVO[n].name, first, (unsigned)us(first), p->deg);

    uint32_t over[SERVO_COUNT] = { 0 }, fast[SERVO_COUNT] = { 0 };
    uint32_t t = hw_millis(), t_end = t + PROBE_MS;
    bool     released = !hw_button_down();  /* the press that started the session may still be held */
    while ((int32_t)(t - t_end) < 0) {
        hw_wait_until(++t);
        float d_ref[SERVO_COUNT];
        int   k = 0;
        for (int m = 0; m < SERVO_COUNT; m++) if (refm[m]) d_ref[k++] = absf(pot_fast_deg(m) - ref0[m]);
        float cm = k ? median(d_ref, k) : 0.0f;
        for (int m = 0; m < SERVO_COUNT; m++) {
            if (!watch[m]) continue;
            float now_m = pot_fast_deg(m);
            if (absf(now_m - prev[m]) > PROBE_GLITCH_DEG_MS) continue;  /* electrical, not motion */
            prev[m] = now_m;
            float d = absf(now_m - ref0[m]);
            if (d > peak) peak = d;
            bool electrical = cm > 1.0f && cm > 0.5f * d;
            over[m] = (!electrical && d > PROBE_CUT_DEG)  ? over[m] + 1u : 0u;
            fast[m] = (!electrical && d > PROBE_FAST_DEG) ? fast[m] + 1u : 0u;
            if (over[m] >= PROBE_SUSTAIN_MS || fast[m] >= PROBE_FAST_MS) {
                hw_set_pulse_us(n, 0);
                s->on = false;
                con_printf("  %s: the pot of %s moved %.1f deg on the first pulse -> pulse cut at once\r\n",
                           SERVO[n].name, SERVO[m].name, d);
                con_printf("     A few degrees: set CFG_OFFSET_HINT of %s to about %+.1f. A jump: the servo does\r\n"
                           "     not sit where its pot says (feedback wire, or run leg_lab PROGRAM_POTCAL on it).\r\n",
                           SERVO[n].name, pot_fast_deg(n) - first);
                note_stop("switch-on: servo moved on its first pulse", n);
                next_tick = hw_millis();
                return R_CUT;
            }
        }
        if (!released) {
            released = !hw_button_down();
        } else if (hw_button_down()) {
            hw_set_pulse_us(n, 0);
            s->on = false;
            con_puts("  stopped by USER during switch-on: pulse cut\r\n");
            note_stop("stopped by user during power-up", n);
            next_tick = hw_millis();
            return R_FROZEN;
        }
    }
    next_tick = hw_millis();

    result_t r = sv_hold(SETTLE_MS);
    if (r != R_OK) return r;
    float off = p->deg - s->out;
    if (absf(off) > MAX_OFFSET_DEG) {
        con_printf("  %s: holds %.1f deg away from where its pot said -> stop\r\n", SERVO[n].name, off);
        note_stop("switch-on: servo does not hold where its pot says", n);
        return R_CUT;
    }
    s->offset = off;
    s->offset_known = true;

    float gain = 1.0f;
#if POWERUP_WIGGLE
    /* the pot must follow a slow +-2 deg move, or it is not this servo's pot */
    float to[SERVO_COUNT], p_hi, p_lo;
    for (int m = 0; m < SERVO_COUNT; m++) to[m] = sv[m].cmd;
    to[n] = first + WIGGLE_DEG;
    if ((r = sv_move(to, WIGGLE_SPEED_DEG_S)) != R_OK || (r = sv_hold(250)) != R_OK) return r;
    p_hi = p->deg;
    to[n] = first - WIGGLE_DEG;
    if ((r = sv_move(to, WIGGLE_SPEED_DEG_S)) != R_OK || (r = sv_hold(250)) != R_OK) return r;
    p_lo = p->deg;
    to[n] = first;
    if ((r = sv_move(to, WIGGLE_SPEED_DEG_S)) != R_OK) return r;
    gain = (p_hi - p_lo) / (2.0f * WIGGLE_DEG);
    if (gain < WIGGLE_MIN_GAIN) {
        con_printf("  %s: pot moved %.1f deg for a %.0f deg move -> not following\r\n", SERVO[n].name, p_hi - p_lo,
                   2.0f * WIGGLE_DEG);
        note_stop("switch-on: pot does not follow the servo", n);
        return R_CUT;
    }
#endif
    con_printf("  %s: on at %.1f deg (q %+.1f), pot offset %+.1f (hint %+.1f), wiggle gain %.2f, probe move %.1f\r\n",
               SERVO[n].name, s->out, pose_q(n, s->out), off, OFFSET_HINT[n], gain, peak);
    return R_OK;
}

result_t sv_power_up(void)
{
    /* every servo is checked before the first pulse */
    int bad = 0;
    for (int n = 0; n < SERVO_COUNT; n++) {
        float first = pot[n].deg - OFFSET_HINT[n];
        const char *why;
        if (!pot_trusted(n))           why = "pot UNTRUSTED: run leg_lab PROGRAM_POTCAL on it, then set POT_LINE";
        else if (!pose_calibrated(n))  why = "calibrator pose not set (CFG_CAL_POSE)";
        else if (!pot[n].valid)        why = "pot reading invalid";
        else                           why = pose_check_start(n, pose_q(n, first), first);
        if (why) {
            con_printf("  %s: %s (pot %.1f deg, q %+.1f)\r\n", SERVO[n].name, why, pot[n].deg, pose_q(n, first));
            bad++;
        }
    }
    if (bad) {
        note_stop("power-up refused", -1);
        return R_REFUSED;
    }

    vdda_base = pot_vdda;
    static const uint8_t LEG_ORDER[LEG_COUNT] = { LEG_FR, LEG_FL, LEG_BR, LEG_BL };
    for (int l = 0; l < LEG_COUNT; l++) {
        for (int j = 0; j < NJ; j++) {
            int n = LEG[LEG_ORDER[l]].servo[j];
            sv_set_state('P', 0.0);
            result_t r = switch_on(n);
            if (r != R_OK) {
                if (sv_any_on()) sv_cut_all(r == R_FROZEN ? "stopped by user during power-up" : "power-up failed");
                return R_CUT;
            }
            vdda_base = pot_vdda;           /* servo current may lower VDDA a little for good */
        }
    }
    return R_OK;
}
