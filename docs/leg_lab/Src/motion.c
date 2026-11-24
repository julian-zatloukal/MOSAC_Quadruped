#include "motion.h"
#include "config.h"
#include "console.h"
#include "feedback.h"
#include "hw.h"
#include <string.h>

joint_t mo_j[NJ];
float   mo_track_max[NJ];

const char *const SEARCH_KIND_NAME[] = {
    "contact", "contact-other-joint", "ran-ahead", "window-edge", "electrical", "stopped", "cut",
};

static const float OFFSET_HINT[SERVO_COUNT] = CFG_OFFSET_HINT;

static uint32_t next_tick, next_telemetry;
static bool     running, parking, stop_req, cut_happened;
static int      last_char;
static bool     last_pressed;
static uint8_t  btn_count;
static bool     btn_was_down;
static bool     disturbed;
static uint32_t disturb_ms, calm_ms;
static char     last_stop[128] = "none since reset";

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float absf(float v) { return v < 0.0f ? -v : v; }
static const float DT = (float)TICK_MS / 1000.0f;

/* --------------------------------------------------------------- basics --- */

void mo_init(void)
{
    for (int j = 0; j < NJ; j++) {
        memset(&mo_j[j], 0, sizeof mo_j[j]);
        mo_j[j].servo = LEG[TEST_LEG].servo[j];
        mo_j[j].lo    = ABS_MIN_DEG;
        mo_j[j].hi    = ABS_MAX_DEG;
    }
    next_tick = next_telemetry = hw_millis();
}

void mo_begin_program(void)
{
    running = true;
    parking = stop_req = cut_happened = false;
    disturbed = false;
    disturb_ms = calm_ms = 0;
    strcpy(last_stop, "running");
    next_tick = next_telemetry = hw_millis();
}

void mo_end_program(void) { running = false; parking = false; stop_req = false; }

int  mo_char(void)    { return last_char; }
bool mo_pressed(void) { return last_pressed; }
bool mo_was_cut(void) { return cut_happened; }

uint16_t mo_us(float deg)
{
    /* 0..266 deg = 500..2400 us, the range the pot tables were measured over. The
     * joint windows (ABS_MIN_DEG..ABS_MAX_DEG at most) keep normal programs inside;
     * only PROGRAM_POTCAL, with the horn off, uses the ends. */
    deg = clampf(deg, 0.0f, 266.0f);
    return (uint16_t)(US_AT_0_DEG + deg * US_PER_DEG + 0.5f);
}

float mo_meas(int j) { return fb_pot[mo_j[j].servo].deg - mo_j[j].offset; }
float mo_err(int j)  { return mo_meas(j) - mo_j[j].out; }

bool mo_any_on(void)
{
    for (int j = 0; j < NJ; j++) if (mo_j[j].on) return true;
    return false;
}

void mo_note_stop(const char *why, int servo)
{
    size_t n = strlen(why);
    if (n > sizeof last_stop - 16) n = sizeof last_stop - 16;
    memcpy(last_stop, why, n);
    last_stop[n] = '\0';
    if (servo >= 0) {
        strcat(last_stop, " (");
        strcat(last_stop, SERVO[servo].name);
        strcat(last_stop, ")");
    }
}

const char *mo_last_stop(void) { return last_stop; }

void mo_cut(int j)
{
    joint_t *J = &mo_j[j];
    J->on = false;
    J->offset_known = false;
    J->exempt = false;
    hw_set_pulse_us(J->servo, 0);
}

void mo_cut_all(const char *why, int servo)
{
    for (int j = 0; j < NJ; j++) mo_cut(j);
    if (!cut_happened) {
        mo_note_stop(why, servo);
        con_printf("CUT: %s%s%s -> all pulses off\r\n", why, servo >= 0 ? " " : "",
                   servo >= 0 ? SERVO[servo].name : "");
    }
    cut_happened = true;
    hw_led(HW_LED_GREEN, false);
    hw_led(HW_LED_RED, true);
}

void mo_freeze(int j)
{
    joint_t *J = &mo_j[j];
    if (!J->on) return;
    /* Command the joint where it really is: the servo stops pushing. During an
     * electrical disturbance the readings are not trusted, and a reading far
     * from the command is a fault, not a position: then just stop advancing. */
    float at = J->out;
    if (J->offset_known && !disturbed && fb_pot[J->servo].valid) {
        float m = mo_meas(j);
        if (absf(m - J->out) < FREEZE_MAX_DEG) at = m;
    }
    J->cmd = J->out = clampf(at, J->lo, J->hi);
}

void mo_freeze_all(void) { for (int j = 0; j < NJ; j++) mo_freeze(j); }

void mo_set_window(int j, float lo, float hi)
{
    joint_t *J = &mo_j[j];
    lo = clampf(lo, ABS_MIN_DEG, ABS_MAX_DEG);
    hi = clampf(hi, ABS_MIN_DEG, ABS_MAX_DEG);
    /* The window always contains where the joint is commanded now, so setting it
     * can never make the command jump. */
    if (J->on) {
        if (lo > J->out) lo = J->out;
        if (hi < J->out) hi = J->out;
    }
    J->lo = lo;
    J->hi = hi;
    J->cmd = clampf(J->cmd, lo, hi);
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
    if (!mo_any_on()) { disturbed = false; disturb_ms = calm_ms = 0; return; }

    float rs  = fb_ref_shift();
    float dip = fb_vdda_dip();
    if (fb_vdda < VDDA_MIN_V) {
        con_printf("supply: VDDA %.2f V (Nucleo close to brown-out)\r\n", fb_vdda);
        mo_cut_all("supply: VDDA too low", -1);
        return;
    }
    if (rs > REF_ABORT_DEG) {
        con_printf("electrical: reference pots moved %.1f deg\r\n", rs);
        mo_cut_all("electrical disturbance: reference pots of the other legs moved", -1);
        return;
    }
    if (rs > REF_PAUSE_DEG || dip > VDDA_DIP_V) {
        if (!disturbed)
            con_printf("electrical disturbance: reference pots moved %.1f deg, VDDA dip %.3f V -> pause\r\n", rs, dip);
        disturbed = true;
        calm_ms = 0;
    } else if (disturbed) {
        calm_ms = (rs < REF_CLEAR_DEG && dip < 0.5f * VDDA_DIP_V) ? calm_ms + TICK_MS : 0;
        if (calm_ms >= 300u) {
            disturbed = false;
            disturb_ms = 0;
            con_puts("electrical disturbance cleared\r\n");
        }
    }
    if (disturbed) {
        disturb_ms += TICK_MS;
        if (disturb_ms > DISTURB_MAX_MS) { mo_cut_all("electrical disturbance did not clear", -1); return; }
    }

    for (int j = 0; j < NJ; j++) {
        joint_t *J = &mo_j[j];
        if (!J->on || !J->offset_known) continue;
        const pot_t *p = &fb_pot[J->servo];
        if (!p->valid) {
            J->invalid_ms += TICK_MS;
            if (J->invalid_ms > 100u) { mo_cut_all("pot reading became invalid while powered", J->servo); return; }
            continue;
        }
        J->invalid_ms = 0;
        float e = absf(mo_err(j));
        if (J->exempt) {
            if (e > EXEMPT_ABORT_DEG) { mo_cut_all("search did not stop: joint far behind its command", J->servo); return; }
            continue;
        }
        J->err_avg += (e - J->err_avg) * ((float)TICK_MS / FAULT_TAU_MS);
        if (J->err_avg > FAULT_ERR_DEG) {
            con_printf("%s not following: sent %.1f, at %.1f, averaged error %.1f deg\r\n",
                       SERVO[J->servo].name, J->out, mo_meas(j), J->err_avg);
            mo_cut_all("fault cut-out: joint not following its command", J->servo);
            return;
        }
    }
}

static void output_update(void)
{
    const float step = MAX_SLEW_DEG_S * DT;
    for (int j = 0; j < NJ; j++) {
        joint_t *J = &mo_j[j];
        if (!J->on) { hw_set_pulse_us(J->servo, 0); continue; }
        J->cmd  = clampf(J->cmd, J->lo, J->hi);
        J->out += clampf(J->cmd - J->out, -step, step);
        hw_set_pulse_us(J->servo, mo_us(J->out));
    }
}

void mo_telemetry_now(void)
{
    con_printf("fb %.1f", (float)hw_millis() / 1000.0f);
    for (int j = 0; j < NJ; j++) {
        const joint_t *J = &mo_j[j];
        const pot_t   *p = &fb_pot[J->servo];
        if (J->on && J->offset_known)
            con_printf(" %s %.1f/%.1f/%+.1f", SERVO[J->servo].name, J->out, mo_meas(j), mo_err(j));
        else if (J->on)
            con_printf(" %s %.1f/pot %.1f/?", SERVO[J->servo].name, J->out, p->deg);
        else
            con_printf(" %s off/%.1f", SERVO[J->servo].name, p->deg);
    }
    con_printf(" adc %u/%u/%u ref %.1f cm %+.1f vdda %.2f%s\r\n",
               (unsigned)fb_pot[mo_j[0].servo].raw, (unsigned)fb_pot[mo_j[1].servo].raw,
               (unsigned)fb_pot[mo_j[2].servo].raw, fb_ref_shift(), fb_common_deg(), fb_vdda,
               disturbed ? " PAUSED" : "");
}

mo_tick_t mo_tick(void)
{
    hw_wait_until(next_tick);
    uint32_t now = hw_millis();
    next_tick += TICK_MS;
    if ((int32_t)(now - next_tick) > 50) next_tick = now + TICK_MS;   /* fell behind (long print): resync */

    int c = hw_uart_getc();
    last_char    = c > 0 ? c : 0;
    last_pressed = button_edge();
    fb_update();

    if (running && mo_any_on()) {
        bool stop_key = last_pressed || last_char == 'x' || last_char == 'X';
        if (last_char == 'k' || last_char == 'K') {
            mo_cut_all("cut by user ('k')", -1);
        } else if (stop_key) {
            if (parking || stop_req) {
                mo_cut_all("cut by user (second stop)", -1);
            } else {
                stop_req = true;
                con_puts("stop requested: easing back to the rest pose. USER or 'x' again cuts at once.\r\n");
            }
        }
    }

    safety();
    output_update();

    if (running && mo_any_on() && (int32_t)(now - next_telemetry) >= 0) {
        next_telemetry = now + TELEMETRY_MS;
        mo_telemetry_now();
    }

    if (cut_happened)          return MO_CUT;
    if (stop_req && !parking)  return MO_STOP;
    if (disturbed)             return MO_PAUSE;
    return MO_OK;
}

/* ------------------------------------------------------------ planned moves --- */

static float ease(float t)   /* smoothstep: zero speed at both ends, peak 1.5x mean */
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

result_t mo_hold(uint32_t ms)
{
    uint32_t t0 = hw_millis();
    while (hw_millis() - t0 < ms) {
        mo_tick_t st = mo_tick();
        if (st == MO_CUT)  return R_CUT;
        if (st == MO_STOP) return R_STOPPED;
    }
    return R_OK;
}

/* Eased move of every powered joint to `to` (clamped to its window), the joint
 * moving furthest peaking at `speed`. Stops pushing and returns R_BLOCKED if a
 * joint stays MOVE_BLOCK_DEG behind for MOVE_BLOCK_MS. Pauses on disturbances. */
result_t mo_move(const float to_in[NJ], float speed, uint32_t hold_ms)
{
    float from[NJ], to[NJ], maxd = 0.0f;
    for (int j = 0; j < NJ; j++) {
        from[j] = mo_j[j].cmd;
        to[j]   = mo_j[j].on ? clampf(to_in[j], mo_j[j].lo, mo_j[j].hi) : mo_j[j].cmd;
        if (absf(to[j] - from[j]) > maxd) maxd = absf(to[j] - from[j]);
    }
    float dur = 0.0f;
    if (maxd > 0.05f) {
        dur = 1.5f * maxd / speed;
        if (dur < (float)MIN_MOVE_MS / 1000.0f) dur = (float)MIN_MOVE_MS / 1000.0f;
    }
    float    p = dur > 0.0f ? 0.0f : 1.0f;
    uint32_t held = 0, blk[NJ] = { 0 };

    for (;;) {
        mo_tick_t st = mo_tick();
        if (st == MO_CUT) return R_CUT;
        if (st == MO_STOP) { mo_freeze_all(); return R_STOPPED; }
        if (st == MO_OK) {
            if (p < 1.0f) {
                p += DT / dur;
                float k = ease(p);
                for (int j = 0; j < NJ; j++) if (mo_j[j].on) mo_j[j].cmd = from[j] + (to[j] - from[j]) * k;
            } else {
                held += TICK_MS;
            }
        }
        for (int j = 0; j < NJ; j++) {
            const joint_t *J = &mo_j[j];
            if (!J->on || !J->offset_known || J->exempt) continue;
            float e = absf(mo_err(j));
            if (e > mo_track_max[j]) mo_track_max[j] = e;
            blk[j] = e > MOVE_BLOCK_DEG ? blk[j] + TICK_MS : 0;
            if (blk[j] >= MOVE_BLOCK_MS) {
                con_printf("%s blocked: sent %.1f, at %.1f deg -> stop pushing\r\n",
                           SERVO[J->servo].name, J->out, mo_meas(j));
                mo_freeze_all();
                mo_note_stop("move blocked", J->servo);
                return R_BLOCKED;
            }
        }
        if (p >= 1.0f && held >= hold_ms) return R_OK;
    }
}

result_t mo_move_joint(int j, float to, float speed, uint32_t hold_ms)
{
    float t[NJ];
    for (int k = 0; k < NJ; k++) t[k] = mo_j[k].cmd;
    t[j] = to;
    return mo_move(t, speed, hold_ms);
}

/* ---------------------------------------------------------------- switch-on --- */

result_t mo_switch_on(int j)
{
    joint_t *J = &mo_j[j];
    int n = J->servo;
    const pot_t *p = &fb_pot[n];

    if (!fb_trusted(n)) {
        con_printf("  %s: pot model is UNTRUSTED -> not switched on (run PROGRAM_POTCAL first)\r\n", SERVO[n].name);
        mo_note_stop("switch-on refused: untrusted pot", n);
        return R_REFUSED;
    }
    if (!p->valid) {
        con_printf("  %s: pot reading invalid (adc %u, spread %u) -> not switched on\r\n",
                   SERVO[n].name, (unsigned)p->raw, (unsigned)p->iqr);
        mo_note_stop("switch-on refused: invalid pot reading", n);
        return R_REFUSED;
    }
    if (disturbed) {
        con_printf("  %s: electrical disturbance in progress -> not switched on\r\n", SERVO[n].name);
        mo_note_stop("switch-on refused: electrical disturbance", n);
        return R_REFUSED;
    }
    float first = p->deg - OFFSET_HINT[n];
    if (first < ABS_MIN_DEG || first > ABS_MAX_DEG) {
        con_printf("  %s: at %.1f deg, outside %.0f..%.0f -> not switched on\r\n",
                   SERVO[n].name, first, ABS_MIN_DEG, ABS_MAX_DEG);
        mo_note_stop("switch-on refused: outside the pulse range", n);
        return R_REFUSED;
    }

    /* Probe: first pulse where the pot says the servo is, all pots of the leg
     * watched every 1 ms. Movement beyond PROBE_CUT_DEG for PROBE_SUSTAIN_MS (or
     * PROBE_FAST_DEG for PROBE_FAST_MS) cuts the pulse at once. A shift that the
     * reference pots of the other legs show too is electrical and is ignored. */
    float ref[NJ];
    bool  use[NJ];
    for (int k = 0; k < NJ; k++) {
        int m  = mo_j[k].servo;
        use[k] = fb_pot[m].valid;
        ref[k] = use[k] ? fb_fast_deg(m) : 0.0f;
    }
    J->cmd = J->out = J->rest = first;
    J->offset = OFFSET_HINT[n];
    J->offset_known = false;
    J->exempt = false;
    J->err_avg = 0.0f;
    J->invalid_ms = 0;
    if (J->lo > first) J->lo = first;
    if (J->hi < first) J->hi = first;
    J->on = true;
    hw_set_pulse_us(n, mo_us(first));
    con_printf("  %s: first pulse at %.1f deg (%u us), pot %.1f\r\n", SERVO[n].name, first,
               (unsigned)mo_us(first), p->deg);

    uint32_t over[NJ] = { 0 }, fast[NJ] = { 0 };
    float    peak[NJ] = { 0 }, prev[NJ];
    uint32_t t = hw_millis(), t_end = t + PROBE_MS;
    bool     released = !hw_button_down();  /* the press that started the program may still be held */
    for (int k = 0; k < NJ; k++) prev[k] = ref[k];
    while ((int32_t)(t - t_end) < 0) {
        hw_wait_until(++t);
        float cm = fb_ref_shift_fast();
        for (int k = 0; k < NJ; k++) {
            if (!use[k]) continue;
            float now_k = fb_fast_deg(mo_j[k].servo);
            if (absf(now_k - prev[k]) > PROBE_GLITCH_DEG_MS) continue;   /* electrical, not motion */
            prev[k] = now_k;
            float d = absf(now_k - ref[k]);
            if (d > peak[k]) peak[k] = d;
            bool electrical = cm > 1.0f && cm > 0.5f * d;
            over[k] = (!electrical && d > PROBE_CUT_DEG)  ? over[k] + 1u : 0u;
            fast[k] = (!electrical && d > PROBE_FAST_DEG) ? fast[k] + 1u : 0u;
            if (over[k] >= PROBE_SUSTAIN_MS || fast[k] >= PROBE_FAST_MS) {
                hw_set_pulse_us(n, 0);
                J->on = false;
                con_printf("  %s: the pot of %s moved %.1f deg on the first pulse -> pulse cut at once\r\n",
                           SERVO[n].name, SERVO[mo_j[k].servo].name, d);
                con_printf("     If it moved only a few degrees, set CFG_OFFSET_HINT of %s to about %+.1f. If\r\n"
                           "     it jumped, the servo does not sit where its pot says: check the feedback wire,\r\n"
                           "     or run PROGRAM_POTCAL on it.\r\n",
                           SERVO[n].name, fb_fast_deg(n) - first);
                mo_note_stop("switch-on: servo moved on its first pulse", n);
                next_tick = hw_millis();
                return R_CUT;
            }
        }
        if (!released) {
            released = !hw_button_down();
        } else if (hw_button_down()) {
            hw_set_pulse_us(n, 0);
            J->on = false;
            con_puts("  stopped by USER during switch-on: pulse cut\r\n");
            mo_note_stop("stopped by user during switch-on", n);
            next_tick = hw_millis();
            return R_STOPPED;
        }
    }
    next_tick = hw_millis();

    result_t r = mo_hold(SETTLE_MS);
    if (r != R_OK) return r;
    float off = fb_pot[n].deg - J->out;
    if (absf(off) > MAX_OFFSET_DEG) {
        con_printf("  %s: holds %.1f deg away from its command -> cut\r\n", SERVO[n].name, off);
        mo_cut(j);
        mo_note_stop("switch-on: servo does not hold where its pot says", n);
        return R_CUT;
    }
    J->offset = off;
    J->offset_known = true;

    /* Wiggle: the pot must follow a slow +-2 deg move, or it is not this servo's
     * pot (stuck, unpowered, or wired to another servo). */
    float p_hi, p_lo;
    if ((r = mo_move_joint(j, first + WIGGLE_DEG, WIGGLE_SPEED_DEG_S, 300)) != R_OK) return r;
    p_hi = fb_pot[n].deg;
    if ((r = mo_move_joint(j, first - WIGGLE_DEG, WIGGLE_SPEED_DEG_S, 300)) != R_OK) return r;
    p_lo = fb_pot[n].deg;
    if ((r = mo_move_joint(j, first, WIGGLE_SPEED_DEG_S, 200)) != R_OK) return r;
    float gain = (p_hi - p_lo) / (2.0f * WIGGLE_DEG);
    if (gain < WIGGLE_MIN_GAIN) {
        con_printf("  %s: pot moved %.1f deg for a %.0f deg move -> not following, cut\r\n",
                   SERVO[n].name, p_hi - p_lo, 2.0f * WIGGLE_DEG);
        mo_cut(j);
        mo_note_stop("switch-on: pot does not follow the servo", n);
        return R_CUT;
    }
    con_printf("  %s: on at %.1f deg, pot offset %+.1f (hint %+.1f), wiggle gain %.2f, peak probe move %.1f\r\n",
               SERVO[n].name, J->out, off, OFFSET_HINT[n], gain, peak[j]);
    return R_OK;
}

/* ------------------------------------------------------------------ search --- */

/* Moves joint j at a steady speed in direction dir (+1/-1) until it meets
 * something, then stops pushing (freeze). Contact = the joint's lag behind its
 * command rises above the lag it had while moving freely (the baseline, which
 * includes gravity load). After the first sign of contact the command slows to
 * SEARCH_SLOW_DEG_S, so the push builds up slowly. The held joints are watched
 * too: a push through the leg shows on them. The window [lo, hi] of the joint is
 * never left. */
search_t mo_search(int j, int dir, float speed, float detect_deg)
{
    joint_t *J = &mo_j[j];
    search_t r;
    memset(&r, 0, sizeof r);
    r.other = -1;

    float    held_ref[NJ];
    uint32_t held_over[NJ] = { 0 };
    for (int k = 0; k < NJ; k++) held_ref[k] = mo_j[k].on ? mo_meas(k) : 0.0f;

    float    v = 0.0f, base = 0.0f, t = 0.0f, last_good = mo_meas(j);
    float    rs0 = fb_ref_shift();
    bool     onset = false;
    uint32_t over = 0, cap_ms = 0, ref_over = 0;
    const float accel = speed / SEARCH_RAMP_S;

    J->exempt = true;
    for (;;) {
        mo_tick_t st = mo_tick();
        if (st == MO_CUT)  { r.kind = S_CUT; break; }
        if (st == MO_STOP) { r.kind = S_STOPPED; break; }
        if (st == MO_PAUSE) {
            /* A disturbance during a search is most often the stall current of the
             * joint itself: take it as contact at the last good reading. */
            r.kind = S_ELECTRICAL;
            if (!onset) r.onset = last_good;
            onset = true;
            break;
        }
        t += DT;
        float meas = mo_meas(j);
        float lag  = (float)dir * (J->out - meas);      /* > 0: behind the command */
        float dev  = lag - base;

        if (t < SEARCH_BASE_S) {
            base = lag;                                 /* still ramping up */
        } else {
            if (dev < 0.7f * SEARCH_ONSET_DEG) {
                base += (lag - base) * DT / SEARCH_BASE_TAU_S;
                last_good = meas;
            }
            dev = lag - base;
            if (dev > r.peak_dev) r.peak_dev = dev;
            if (!onset && dev > SEARCH_ONSET_DEG) { onset = true; r.onset = meas; }
            else if (onset && dev < 0.5f * SEARCH_ONSET_DEG) onset = false;
            over = dev > detect_deg ? over + TICK_MS : 0;
            if (over >= SEARCH_DETECT_MS || dev > SEARCH_FAST_DEG) { r.kind = S_CONTACT; break; }
            if (dev < -SEARCH_AHEAD_DEG) { r.kind = S_AHEAD; r.onset = meas; onset = true; break; }
        }

        /* Current drawn by a stalled servo shows on the reference pots. */
        ref_over = fb_ref_shift() - rs0 > SEARCH_REF_DEG ? ref_over + TICK_MS : 0;
        if (ref_over >= 50u) {
            r.kind = S_ELECTRICAL;
            if (!onset) r.onset = last_good;
            onset = true;
            break;
        }

        bool other = false;
        for (int k = 0; k < NJ; k++) {
            if (k == j || !mo_j[k].on || !mo_j[k].offset_known) continue;
            held_over[k] = absf(mo_meas(k) - held_ref[k]) > HELD_DEFLECT_DEG ? held_over[k] + TICK_MS : 0;
            if (held_over[k] >= HELD_DEFLECT_MS) { r.other = k; other = true; }
        }
        if (other) { r.kind = S_CONTACT_OTHER; if (!onset) r.onset = meas; onset = true; break; }

        float vt = onset ? SEARCH_SLOW_DEG_S : speed;
        if (v < vt) { v += accel * DT; if (v > vt) v = vt; }
        else        { v -= accel * DT; if (v < vt) v = vt; }

        if (st == MO_OK) {
            float next = J->cmd + (float)dir * v * DT;
            if (next < J->lo || next > J->hi) {
                J->cmd = dir > 0 ? J->hi : J->lo;
                cap_ms += TICK_MS;
                if ((cap_ms >= SEARCH_CAP_WAIT_MS && !onset) || cap_ms >= 4u * SEARCH_CAP_WAIT_MS) {
                    r.kind = S_CAP;
                    break;
                }
            } else {
                J->cmd = next;
            }
        }
    }
    r.at_end = mo_meas(j);
    if (!onset) r.onset = r.at_end;
    J->exempt = false;
    J->err_avg = 0.0f;
    if (r.kind != S_CUT) {
        mo_freeze(j);
        if (r.kind == S_ELECTRICAL) {
            /* back off a little from the last good reading to release a stall */
            J->cmd = J->out = clampf(last_good - (float)dir * 1.0f, J->lo, J->hi);
        }
    }
    return r;
}

/* ---------------------------------------------------------------- parking --- */

result_t mo_park_joint(int j)
{
    joint_t *J = &mo_j[j];
    if (!J->on) return R_OK;
    result_t r = R_OK;
    if (J->offset_known && !cut_happened) {
        parking = true;
        r = mo_move_joint(j, J->rest, MAX_SPEED_DEG_S, PARK_HOLD_MS);
        parking = false;
    }
    if (J->on) mo_cut(j);
    return r;
}

result_t mo_park(void)
{
    if (!mo_any_on()) return R_OK;
    float to[NJ];
    for (int j = 0; j < NJ; j++) to[j] = mo_j[j].rest;
    parking  = true;
    stop_req = false;
    con_puts("parking: back to the rest pose, then pulses off\r\n");
    result_t r = cut_happened ? R_CUT : mo_move(to, MAX_SPEED_DEG_S, PARK_HOLD_MS);
    parking = false;
    for (int j = 0; j < NJ; j++) if (mo_j[j].on) mo_cut(j);
    con_puts(r == R_OK ? "parked: pulses off\r\n" : "parking interrupted: pulses off\r\n");
    return r;
}
