/*
 * gait_runner: the latest Webots walk (Programmed_Yaw, forward) on the real
 * MOSAC quadruped, NUCLEO-F412ZG. See README.md. Choose RUN_MODE in config.h.
 *
 * Idle (after reset and after every session): no pulses. Pots and joint angles
 * are streamed, and these keys work:
 *   p  configuration and the walk plan      c  capture the calibrator pose (all 12)
 *   +  -  speed scale                        h  help
 *   s / USER  start a session (power up, stand up)
 * A session: every servo switches on where it is (one at a time), the legs ease
 * to a crouch, the robot rises to the standing height, and the stand check runs.
 * Then, standing: USER or 'w' walks (RUN_WALK), 'd' sits down, '+'/'-' speed.
 * Walking: USER or 'x' stops (the legs finish their stage and centre); pressing
 * again freezes. 'k' cuts every pulse at any time (the robot drops).
 */
#include "config.h"
#include "console.h"
#include "gait.h"
#include "gait_params.h"
#include "hw.h"
#include "plan.h"
#include "pose.h"
#include "pots.h"
#include "robot.h"
#include "servo.h"

#if RUN_MODE < RUN_MONITOR || RUN_MODE > RUN_WALK
#error "RUN_MODE must be RUN_MONITOR, RUN_STAND or RUN_WALK"
#endif

static const char *const MODE_NAME[] = { "MONITOR", "STAND", "WALK" };
static const gait_params_t WALK = GAIT_WALK_PARAMS;
static const float DT = (float)TICK_MS / 1000.0f;

static gait_t gen;                  /* the walk; its pose is the robot's pose while standing */
static plan_t plan;
static float  speed = SPEED_SCALE;
static bool   config_ok, walk_ok;
static float  q_stand[SERVO_COUNT], q_crouch[SERVO_COUNT];
static bool   stream_raw;

static float absf(float v) { return v < 0.0f ? -v : v; }

static float ease(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

/* ------------------------------------------------------------ reports --- */

static int config_check(void)
{
    int bad = 0;
    for (int n = 0; n < SERVO_COUNT; n++) {
        if (!pot_trusted(n)) {
            con_printf("  %s: pot UNTRUSTED%s -> leg_lab PROGRAM_POTCAL (horn off), then CFG_POT_MODEL / CFG_POT_LINE\r\n",
                       SERVO[n].name, n == 6 ? " (the swapped femur)" : "");
            bad++;
        }
        if (!pose_calibrated(n)) {
            con_printf("  %s: CFG_CAL_POSE not set -> fit the calibrators, 'c' in RUN_MONITOR\r\n", SERVO[n].name);
            bad++;
        } else if (pose_mid(n) < ABS_MIN_DEG + 20.0f || pose_mid(n) > ABS_MAX_DEG - 20.0f) {
            con_printf("  %s: mid point %.1f deg is near the end of the servo range\r\n", SERVO[n].name, pose_mid(n));
            bad++;
        }
    }
    if (!(H_LOW_MM < WALK.standing_height - 10.0f)) { con_puts("  H_LOW_MM must be below the standing height\r\n"); bad++; }
    bool reach = pose_legs(H_LOW_MM, 0.0f, q_crouch) && pose_legs(WALK.standing_height, 0.0f, q_stand);
    if (!reach) { con_puts("  crouch or standing pose out of reach\r\n"); return bad + 1; }
    for (int n = 0; n < SERVO_COUNT; n++) {
        const char *why = pose_check(n, q_crouch[n], 0);
        if (why) { con_printf("  crouch: %s q %+.1f %s\r\n", SERVO[n].name, q_crouch[n], why); bad++; }
        why = pose_check(n, q_stand[n], 0);
        if (why) { con_printf("  standing pose: %s q %+.1f %s\r\n", SERVO[n].name, q_stand[n], why); bad++; }
    }
    return bad;
}

static void print_config(void)
{
    con_printf("walk: %s, period %.3f s in Webots, stride %.1f mm, lift %.1f mm, height %.1f mm, lean %.1f/%.1f mm\r\n",
               "Programmed_Yaw walk_params.json", (float)WALK.period, WALK.parabola_length, WALK.parabola_height,
               WALK.standing_height, WALK.lean_primary, WALK.lean_secondary);
    con_printf("      stage weights %.3f/%.3f/%.3f/%.3f, ramp %.1f cycles; here: speed %.2f, stride %.2f, %d cycles\r\n",
               (float)WALK.weight[0], (float)WALK.weight[1], (float)WALK.weight[2], (float)WALK.weight[3],
               (float)WALK.ramp_cycles, speed, STRIDE, WALK_CYCLES);
    con_puts("servo  joint  pins      pot    dir  calibrator  mid    crouch stand  (servo deg)\r\n");
    for (int n = 0; n < SERVO_COUNT; n++) {
        con_printf("  %s  %s  %s/%s  %s  %+d  ", SERVO[n].name, JOINT_NAME[SERVO[n].joint], SERVO[n].pwm_pin,
                   SERVO[n].fb_pin, pot_model_name(n), SERVO[n].dir);
        if (pose_calibrated(n))
            con_printf("%.2f  %.2f  %.1f  %.1f\r\n", pose_cal_cmd(n), pose_mid(n), pose_deg(n, q_crouch[n]),
                       pose_deg(n, q_stand[n]));
        else
            con_puts("UNSET\r\n");
    }
    con_printf("limits: joint speed %.0f deg/s (50 ms), slew %.0f deg/s walking / %.0f moving; tracking %.0f / %.0f deg\r\n",
               MAX_JOINT_SPEED_DEG_S, SLEW_LIMIT_DEG_S, MOVE_SLEW_DEG_S, TRACK_WALK_DEG, TRACK_MOVE_DEG);
}

static void print_plan(void)
{
    if (!plan.ok) {
        con_printf("walk plan: REFUSED: %s", plan.why ? plan.why : "not computed");
        if (plan.why_servo >= 0) con_printf(" (%s at q %+.1f)", SERVO[plan.why_servo].name, plan.why_q);
        con_puts("\r\n");
        return;
    }
    con_printf("walk plan: speed %.2f%s (period %.2f s), stride %.2f, %d cycles + ramp + stop = %.1f s; peak joint %.0f deg/s"
               " (50 ms), %.0f over one tick\r\n", plan.speed, plan.slowed ? " (lowered for the joint speed limit)" : "",
               (float)WALK.period / plan.speed, STRIDE, WALK_CYCLES, plan.seconds, plan.peak50, plan.peak_tick);
    for (int l = 0; l < LEG_COUNT; l++) {
        con_printf("  %s:", LEG[l].tag);
        for (int j = 1; j < NJ; j++) {
            int n = LEG[l].servo[j];
            con_printf(" %s q %+.1f..%+.1f", SERVO[n].name, plan.q_lo[n], plan.q_hi[n]);
            if (pose_calibrated(n)) con_printf(" (%.1f..%.1f deg)", plan.deg_lo[n], plan.deg_hi[n]);
        }
        con_puts("\r\n");
    }
}

static void replan(void (*yield)(void))
{
    if (!config_ok) { plan.ok = false; plan.why = "configuration problems (listed at start-up)"; plan.why_servo = -1; return; }
    plan_walk(&gen, speed, STRIDE, WALK_CYCLES, &plan, yield);
}

static void change_speed(int c, void (*yield)(void))
{
    speed += (c == '+') ? SPEED_STEP : -SPEED_STEP;
    if (speed < SPEED_MIN) speed = SPEED_MIN;
    if (speed > SPEED_MAX) speed = SPEED_MAX;
    replan(yield);
    print_plan();
}

static void print_help(void)
{
    con_puts("idle: s/USER start  p config+plan  c capture calibrators  +/- speed  a raw ADC  h help\r\n"
             "standing: USER/w walk  d sit down  +/- speed  p plan   walking: USER/x stop (again: freeze)\r\n"
             "any time: k cuts every pulse (the robot drops)\r\n");
}

static void idle_stream(void)
{
    con_printf("pot %.1f", (float)hw_millis() / 1000.0f);
    for (int n = 0; n < SERVO_COUNT; n++) {
        con_printf(" %s %.1f", SERVO[n].name, pot[n].deg);
        if (pose_calibrated(n) && pot_trusted(n)) con_printf(" (q %+.1f)", pose_q(n, pot[n].deg));
        if (!pot[n].valid) con_puts("!");
        if (stream_raw) con_printf(" [%u/%u]", (unsigned)pot[n].raw, (unsigned)pot[n].iqr);
    }
    con_printf(" vdda %.2f\r\n", pot_vdda);
}

/* Average of all 12 pots over 1 s with the printed calibrators fitted. */
static void capture_cal(void)
{
    float sum[SERVO_COUNT] = { 0 }, lo[SERVO_COUNT], hi[SERVO_COUNT];
    for (int n = 0; n < SERVO_COUNT; n++) { lo[n] = 1e9f; hi[n] = -1e9f; }
    con_puts("capturing the calibrator pose of all four legs: hold still for 1 s...\r\n");
    for (int i = 0; i < 100; i++) {
        sv_tick();
        for (int n = 0; n < SERVO_COUNT; n++) {
            float d = pot[n].deg;
            sum[n] += d;
            if (d < lo[n]) lo[n] = d;
            if (d > hi[n]) hi[n] = d;
        }
    }
    for (int n = 0; n < SERVO_COUNT; n++)
        con_printf("  %s %.2f deg (moved %.2f)%s%s\r\n", SERVO[n].name, sum[n] / 100.0f, hi[n] - lo[n],
                   hi[n] - lo[n] > 0.5f ? "  NOT STEADY" : "", pot_trusted(n) ? "" : "  UNTRUSTED pot: capture again after POTCAL");
    con_puts("paste into config.h:\r\n#define CFG_CAL_POSE { \\\r\n   ");
    for (int n = 0; n < SERVO_COUNT; n++) {
        if (pot_trusted(n)) con_printf(" %.2ff", sum[n] / 100.0f);
        else con_puts(" UNSET");
        con_puts(n == SERVO_COUNT - 1 ? " }\r\n" : (n % 6 == 5 ? ", \\\r\n   " : ","));
    }
}

/* ---------------------------------------------------------- sequences --- */

/* Checks a whole pose, then commands it; a target outside the windows is a fault. */
static result_t apply_q(const float q[SERVO_COUNT], const char *what)
{
    float deg[SERVO_COUNT];
    for (int n = 0; n < SERVO_COUNT; n++) {
        const char *why = pose_check(n, q[n], &deg[n]);
        if (why) {
            con_printf("%s: %s target q %+.1f is %s\r\n", what, SERVO[n].name, q[n], why);
            sv_fault("target outside the joint windows", n);
            return R_FAULT;
        }
    }
    for (int n = 0; n < SERVO_COUNT; n++) sv_command(n, deg[n]);
    return R_OK;
}

static result_t move_q(const float q[SERVO_COUNT], const char *what)
{
    float deg[SERVO_COUNT];
    for (int n = 0; n < SERVO_COUNT; n++) {
        const char *why = pose_check(n, q[n], &deg[n]);
        if (why) {
            con_printf("%s: %s target q %+.1f is %s\r\n", what, SERVO[n].name, q[n], why);
            sv_fault("target outside the joint windows", n);
            return R_FAULT;
        }
    }
    con_printf("moving to the %s (peak %.0f deg/s)\r\n", what, MOVE_SPEED_DEG_S);
    sv_set_state('M', 0.0);
    return sv_move(deg, MOVE_SPEED_DEG_S);
}

/* Feet straight under the hips while the height changes from h0 to h1. */
static result_t change_height(float h0, float h1)
{
    float dur = 1.5f * absf(h1 - h0) / H_RATE_MM_S, p = 0.0f;
    if (dur < 0.4f) dur = 0.4f;
    con_printf("%s: hip-to-foot %.0f -> %.0f mm in %.1f s\r\n", h1 > h0 ? "standing up" : "sitting down", h0, h1, dur);
    sv_set_state(h1 > h0 ? 'R' : 'D', 0.0);
    for (;;) {
        tk_t st = sv_tick();
        if (st == TK_CUT)   return R_CUT;
        if (st == TK_FAULT) return R_FAULT;
        if (sv_user_stop()) return R_FROZEN;
        if (st == TK_PAUSE) continue;
        if (p >= 1.0f) return R_OK;
        p += DT / dur;
        if (p > 1.0f) p = 1.0f;
        float q[SERVO_COUNT];
        if (!pose_legs(h0 + (h1 - h0) * ease(p), 0.0f, q)) { sv_fault("height out of reach", -1); return R_FAULT; }
        result_t r = apply_q(q, "height change");
        if (r != R_OK) return r;
    }
}

static result_t stand_check(void)
{
    sv_set_state('S', gen.t);
    result_t r = sv_hold(STAND_SETTLE_MS);
    if (r != R_OK) return r;
    float worst = 0.0f;
    int   wn = 0;
    con_puts("stand check, pot - command (deg):");
    for (int n = 0; n < SERVO_COUNT; n++) {
        float e = sv_err(n);
        con_printf(" %s %+.1f", SERVO[n].name, e);
        if (absf(e) > worst) { worst = absf(e); wn = n; }
    }
    con_printf("  vdda %.2f\r\n", pot_vdda);
    walk_ok = worst <= STAND_ERR_DEG;
    if (walk_ok)
        con_printf("stand check passed: largest error %.1f deg (%s)\r\n", worst, SERVO[wn].name);
    else
        con_printf("stand check FAILED: %s is %.1f deg from its command (limit %.1f): walking disabled.\r\n"
                   "  It does not hold its pose under load, or its calibration is off. 'd' sits down.\r\n",
                   SERVO[wn].name, sv_err(wn), STAND_ERR_DEG);
    return R_OK;
}

static result_t walk(void)
{
    con_puts("dry run of the walk from this pose...\r\n");
    replan(sv_poll);
    if (!sv_any_on()) return R_CUT;
    if (sv_faulted()) return R_FAULT;
    print_plan();
    if (!plan.ok) { con_puts("walk refused\r\n"); return R_OK; }
    (void)sv_take_press();
    (void)sv_take_char();

    con_puts("WALKING. USER or 'x' stops (the legs finish their stage and centre), again freezes; 'k' cuts.\r\n");
    walk_t w;
    walk_begin(&w, &gen, plan.speed, STRIDE, WALK_CYCLES);
    sv_set_limits(TRACK_WALK_DEG, SLEW_LIMIT_DEG_S);
    result_t r = R_OK;
    for (;;) {
        tk_t st = sv_tick();
        if (st == TK_CUT)   { r = R_CUT; break; }
        if (st == TK_FAULT) { r = R_FAULT; break; }
        int c = sv_take_char();
        if (sv_take_press() || c == 'x' || c == 'X') {
            if (!w.stopping) {
                walk_stop(&w, &gen);
                con_puts("stopping: the legs finish their stage and centre. USER again freezes.\r\n");
            } else {
                sv_freeze_by_user();
                r = R_FROZEN;
                break;
            }
        }
        if (st == TK_PAUSE) continue;       /* the gait clock stops; the commands hold */
        bool done = walk_tick(&w, &gen);
        sv_set_state(w.stopping ? 'w' : 'W', gen.t);
        if (!gen.ik_ok) { sv_fault("gait: a foot went out of reach", -1); r = R_FAULT; break; }
        float q[SERVO_COUNT];
        pose_of_gait(&gen, q);
        if ((r = apply_q(q, "walk")) != R_OK) break;
        if (done) break;
    }
    if (r == R_OK) {
        con_puts("walk finished: standing, feet centred\r\n");
        sv_set_state('S', gen.t);
        r = sv_hold(500);                   /* the servos catch up before the tighter limit */
    }
    sv_set_limits(TRACK_MOVE_DEG, MOVE_SLEW_DEG_S);
    return r;
}

typedef enum { EV_WALK, EV_DOWN, EV_FAULT, EV_CUT } ev_t;

static ev_t standing(void)
{
    sv_set_state('S', gen.t);
    if (RUN_MODE == RUN_WALK)
        con_puts("STANDING. USER or 'w' walks, 'd' sits down, '+'/'-' speed, 'p' plan, 'k' cuts.\r\n");
    else
        con_puts("STANDING (RUN_STAND: no walking). USER or 'd' sits down, 'k' cuts.\r\n");
    uint32_t note = hw_millis() + 15000u;
    for (;;) {
        tk_t st = sv_tick();
        if (st == TK_CUT)   return EV_CUT;
        if (st == TK_FAULT) return EV_FAULT;
        int  c = sv_take_char();
        bool pressed = sv_take_press();
        if (RUN_MODE == RUN_WALK && (pressed || c == 'w' || c == 'W')) {
            if (walk_ok) return EV_WALK;
            con_puts("walking disabled: the stand check failed. 'd' sits down.\r\n");
        } else if (RUN_MODE != RUN_WALK && pressed) {
            return EV_DOWN;
        }
        if (c == 'd' || c == 'D') return EV_DOWN;
        if (c == '+' || c == '-') change_speed(c, sv_poll);
        if (c == 'p' || c == 'P') { print_config(); print_plan(); }
        if (c == 'h' || c == '?') print_help();
        if ((int32_t)(hw_millis() - note) >= 0) {
            note = hw_millis() + 15000u;
            con_printf("standing (speed %.2f)\r\n", speed);
        }
    }
}

typedef enum { HE_RECOVER, HE_DOWN, HE_CUT } hold_ev_t;

/* After a fault or a freeze: hold, and let the user decide. Nothing moves by itself. */
static hold_ev_t holding(result_t why)
{
    bool stall_checked = false;
    if (why == R_FAULT) {
        if (sv_fault_hold() == R_CUT) return HE_CUT;
        stall_checked = true;
    }
    sv_set_state('H', gen.t);
    con_printf("HOLDING (%s). USER or 'g' eases back to the standing pose, 'd' sits down, 'k' cuts (the robot drops).\r\n",
               sv_last_stop());
    (void)sv_take_press();                  /* presses made before this message do not count */
    (void)sv_take_char();
    uint32_t note = hw_millis() + 10000u;
    for (;;) {
        tk_t st = sv_tick();
        if (st == TK_CUT) return HE_CUT;
        if (st == TK_FAULT && !stall_checked) {
            if (sv_fault_hold() == R_CUT) return HE_CUT;
            stall_checked = true;
            con_printf("HOLDING (%s)\r\n", sv_last_stop());
        }
        int c = sv_take_char();
        if (sv_take_press() || c == 'g' || c == 'G') return HE_RECOVER;
        if (c == 'd' || c == 'D') return HE_DOWN;
        if ((int32_t)(hw_millis() - note) >= 0) {
            note = hw_millis() + 10000u;
            con_printf("holding (%s): USER/g back to standing, d sit down, k cut\r\n", sv_last_stop());
        }
    }
}

/* From the standing pose: down to the crouch, then wait. */
static result_t sit_and_wait(void)
{
    result_t r = move_q(q_stand, "standing pose");
    if (r == R_OK) r = change_height(WALK.standing_height, H_LOW_MM);
    if (r != R_OK) return r;
    sv_set_state('C', 0.0);
    con_puts("CROUCHED. USER switches off (the robot settles onto its belly), 'u' stands up again, 'k' cuts.\r\n");
    (void)sv_take_press();
    for (;;) {
        tk_t st = sv_tick();
        if (st == TK_CUT)   return R_CUT;
        if (st == TK_FAULT) return R_FAULT;
        int c = sv_take_char();
        if (sv_take_press()) { sv_cut_all("switched off after sitting down"); return R_CUT; }
        if (c == 'u' || c == 'U') {
            r = change_height(H_LOW_MM, WALK.standing_height);
            gait_init(&gen, &WALK);
            return r == R_OK ? stand_check() : r;
        }
    }
}

static void session(void)
{
    if (!config_ok) {
        con_puts("refused: fix the configuration problems listed at start-up ('p' shows the configuration)\r\n");
        return;
    }
    if (RUN_MODE == RUN_WALK && !plan.ok) {
        con_puts("refused: the walk does not pass its dry run ('p' shows the plan)\r\n");
        return;
    }
    con_puts("=== power-up: each servo switches on where it is, one at a time. The robot must be lifted\r\n"
             "    or lying with its legs free. USER stops (cuts everything). ===\r\n");
    sv_begin();
    hw_led(HW_LED_RED, false);
    hw_led(HW_LED_GREEN, true);

    walk_ok = false;
    result_t r = sv_power_up();
    if (r == R_OK) r = move_q(q_crouch, "crouch");
    if (r == R_OK) r = change_height(H_LOW_MM, WALK.standing_height);
    gait_init(&gen, &WALK);
    if (r == R_OK) r = stand_check();

    while (sv_any_on()) {
        if (r == R_CUT) break;
        if (r == R_FAULT || r == R_FROZEN) {
            hold_ev_t ev = holding(r);
            if (ev == HE_CUT) break;
            sv_clear_fault();
            r = move_q(q_stand, "standing pose");
            gait_init(&gen, &WALK);
            if (r == R_OK) r = ev == HE_DOWN ? sit_and_wait() : stand_check();
            continue;
        }
        ev_t ev = standing();
        if (ev == EV_CUT) break;
        if (ev == EV_FAULT) { r = R_FAULT; continue; }
        r = ev == EV_WALK ? walk() : sit_and_wait();
    }
    if (sv_any_on()) sv_cut_all("session ended");
    hw_led(HW_LED_GREEN, false);
    con_printf("=== session ended: %s. All pulses off. ===\r\n", sv_last_stop());
    gait_init(&gen, &WALK);
    replan(0);
}

/* ---------------------------------------------------------------- main --- */

int main(void)
{
    hw_init(RUN_MODE != RUN_MONITOR);
    sv_init();
    pots_update();
    gait_init(&gen, &WALK);

    con_printf("\r\n=== gait_runner, RUN_%s ===\r\n", MODE_NAME[RUN_MODE]);
    if (RUN_MODE == RUN_MONITOR) con_puts("*** NO PWM: the servo signal pins are held low in this build ***\r\n");
    con_printf("clock: 96 MHz from %s\r\n", hw_clock_from_hsi() ? "HSI (no ST-LINK MCO)" : "ST-LINK 8 MHz MCO");
    con_puts("configuration check:\r\n");
    int bad = config_check();
    config_ok = bad == 0;
    if (config_ok) con_puts("  ok\r\n");
    else con_printf("  %d problem(s): sessions are refused until they are fixed\r\n", bad);
    print_config();
    replan(0);
    print_plan();
    print_help();

    uint32_t next_stream = hw_millis();
    for (;;) {
        sv_tick();
        uint32_t now = hw_millis();
        int  c = sv_take_char();
        bool pressed = sv_take_press();
        hw_led(HW_LED_BLUE, (now / 500u) & 1u);
        switch (c) {
        case 'p': case 'P': print_config(); print_plan(); break;
        case 'c': case 'C': capture_cal(); break;
        case 'a': case 'A': stream_raw = !stream_raw; break;
        case '+': case '-': change_speed(c, 0); break;
        case 'h': case 'H': case '?': print_help(); break;
        default: break;
        }
        if (pressed || c == 's' || c == 'S') {
            if (RUN_MODE == RUN_MONITOR) con_puts("RUN_MONITOR: nothing to start (no pulses in this build)\r\n");
            else session();
            next_stream = hw_millis() + 3000u;
        }
        if ((int32_t)(now - next_stream) >= 0) {
            next_stream = now + IDLE_STREAM_MS;
            idle_stream();
        }
    }
}
