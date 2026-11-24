/*
 * leg_lab: single-leg test firmware for the MOSAC quadruped (NUCLEO-F412ZG).
 * Successor of servo_test_leg (docs/servo_test_leg). Choose the program and the
 * leg in config.h. See README.md.
 *
 * Idle (after reset, and after every program): no pulses at all. The pots of the
 * leg are streamed, and these commands work in every build:
 *   r   range of each pot since the last 'r' (move the joints BY HAND first)
 *   c   capture the calibrator pose (fit the printed calibrator, then press)
 *   a   raw ADC of all 12 channels on/off
 *   p   print the configuration and the windows
 *   h   help
 *   s / USER   start the program (PROGRAM_MONITOR never moves anything)
 * While a program runs: USER or 'x' eases back to the rest pose and stops;
 * pressing again, or 'k', cuts all pulses at once.
 */
#include "config.h"
#include "console.h"
#include "feedback.h"
#include "guard.h"
#include "hw.h"
#include "motion.h"
#include "programs.h"
#include "robot.h"

#if TEST_LEG < LEG_FR || TEST_LEG > LEG_BL
#error "TEST_LEG must be LEG_FR, LEG_FL, LEG_BR or LEG_BL"
#endif
#if PROGRAM < PROGRAM_MONITOR || PROGRAM > PROGRAM_LIMITS
#error "PROGRAM must be one of the PROGRAM_x names in config.h"
#endif

static const char *const PROGRAM_NAME[] = { "MONITOR", "CHECK", "POTCAL", "LIMITS" };
static const char *const PROGRAM_TEXT[] = {
    "feedback only: no pulse can be sent in this build",
    "one servo at a time, small slow sweeps around where it is",
    "one servo with its horn OFF, full pulse sweep, new pot table",
    "guarded search of every joint limit, then femur x tibia and femur x body maps",
};

static float rng_lo[SERVO_COUNT], rng_hi[SERVO_COUNT];
static bool  print_all_raw;

/* ---------------------------------------------------------------- idle --- */

static void range_reset(void)
{
    for (int n = 0; n < SERVO_COUNT; n++) rng_lo[n] = rng_hi[n] = fb_pot[n].deg;
}

static void range_update(void)
{
    for (int n = 0; n < SERVO_COUNT; n++) {
        if (fb_pot[n].iqr > FB_MAX_IQR || fb_pot[n].raw < 50u || fb_pot[n].raw > 4045u) continue;
        if (fb_pot[n].deg < rng_lo[n]) rng_lo[n] = fb_pot[n].deg;
        if (fb_pot[n].deg > rng_hi[n]) rng_hi[n] = fb_pot[n].deg;
    }
}

static void print_ranges(void)
{
    const leg_info_t *L = &LEG[TEST_LEG];
    con_printf("hand range of the %s leg since the last 'r' (pot deg):\r\n", L->name);
    for (int j = 0; j < NJ; j++) {
        int n = L->servo[j];
        con_printf("  %s %s %.1f .. %.1f (span %.1f), now %.1f%s\r\n", SERVO[n].name, JOINT_NAME[j], rng_lo[n],
                   rng_hi[n], rng_hi[n] - rng_lo[n], fb_pot[n].deg,
                   fb_trusted(n) ? "" : "  (UNTRUSTED pot: nominal scale, measure again after PROGRAM_POTCAL)");
        con_printf("RES,hand,%s,%s,%.2f,%.2f\r\n", L->tag, SERVO[n].name, rng_lo[n], rng_hi[n]);
    }
    con_puts("paste into config.h:\r\n#define CFG_HAND_RANGE { \\\r\n   ");
    for (int n = 0; n < SERVO_COUNT; n++) {
        bool mine = SERVO[n].leg == TEST_LEG;
        float lo = mine ? rng_lo[n] : guard_hand(n, 0), hi = mine ? rng_hi[n] : guard_hand(n, 1);
        if (lo < UNSET + 1.0f) con_puts(" {UNSET,UNSET}");
        else con_printf(" {%.1ff,%.1ff}", lo, hi);
        con_puts(n == SERVO_COUNT - 1 ? " }\r\n" : (n % 4 == 3 ? ", \\\r\n   " : ","));
    }
    range_reset();
}

/* Average of the leg's pots over 1 s, which must be steady. */
static void capture_ref(void)
{
    const leg_info_t *L = &LEG[TEST_LEG];
    float sum[NJ] = { 0 }, lo[NJ], hi[NJ];
    for (int j = 0; j < NJ; j++) { lo[j] = 1e9f; hi[j] = -1e9f; }
    con_printf("capturing the calibrator pose of the %s leg: hold still for 1 s...\r\n", L->name);
    for (int i = 0; i < 100; i++) {
        mo_tick();
        for (int j = 0; j < NJ; j++) {
            float d = fb_pot[L->servo[j]].deg;
            sum[j] += d;
            if (d < lo[j]) lo[j] = d;
            if (d > hi[j]) hi[j] = d;
        }
    }
    float mean[NJ];
    for (int j = 0; j < NJ; j++) {
        int n = L->servo[j];
        mean[j] = sum[j] / 100.0f;
        con_printf("  %s %.2f deg (adc %u, moved %.2f during the capture)%s%s\r\n", SERVO[n].name, mean[j],
                   (unsigned)fb_pot[n].raw, hi[j] - lo[j], hi[j] - lo[j] > 0.5f ? "  NOT STEADY" : "",
                   fb_trusted(n) ? "" : "  UNTRUSTED pot: capture again after PROGRAM_POTCAL");
        con_printf("RES,ref,%s,%s,%.2f\r\n", L->tag, SERVO[n].name, mean[j]);
    }
    con_puts("paste into config.h:\r\n#define CFG_REF_POSE { \\\r\n   ");
    for (int n = 0; n < SERVO_COUNT; n++) {
        float v = guard_ref(n);
        for (int j = 0; j < NJ; j++) if (L->servo[j] == n) v = mean[j];
        if (v < UNSET + 1.0f) con_puts(" UNSET");
        else con_printf(" %.2ff", v);
        con_puts(n == SERVO_COUNT - 1 ? " }\r\n" : (n % 6 == 5 ? ", \\\r\n   " : ","));
    }
}

static void print_config(void)
{
    static const float TRAVEL[NJ] = LIMIT_TRAVEL_DEG;
    const leg_info_t *L = &LEG[TEST_LEG];
    con_printf("program %s, leg %s\r\n", PROGRAM_NAME[PROGRAM], L->name);
    for (int j = 0; j < NJ; j++) {
        int n = L->servo[j];
        con_printf("  %-5s %s (%s): PWM TIM%d CH%d %s, pot ADC1 IN%d %s, model %s, dir %+d", JOINT_NAME[j],
                   SERVO[n].name, SERVO[n].label, n / 4 + 2, n % 4 + 1, SERVO[n].pwm_pin, SERVO[n].adc_ch,
                   SERVO[n].fb_pin, fb_model_name(n), SERVO[n].dir);
        if (guard_has_hand(n)) con_printf(", hand %.1f..%.1f", guard_hand(n, 0), guard_hand(n, 1));
        if (guard_has_ref(n))  con_printf(", calibrator %.1f (q now %.1f)", guard_ref(n), guard_q_of_pot(n, fb_pot[n].deg));
        con_puts("\r\n");
        window_t w = guard_window(n, j, fb_pot[n].deg, TRAVEL[j], 0.0f);
        con_puts("  ");
        guard_print(n, &w, 0.0f);
    }
    con_printf("speeds: moves %.0f deg/s peak, slew limit %.0f deg/s, searches %.0f -> %.0f deg/s\r\n",
               MAX_SPEED_DEG_S, MAX_SLEW_DEG_S, SEARCH_SPEED_DEG_S, SEARCH_SLOW_DEG_S);
    con_printf("electrical watch: %d reference pots, VDDA %.2f V\r\n", fb_ref_count(), fb_vdda);
}

static void print_help(void)
{
    con_puts("commands: s/USER start, r hand range, c capture calibrator pose, a raw ADC of all, p config, h help\r\n"
             "while running: USER/x ease back and stop (again = cut at once), k cut at once\r\n");
}

static void idle_stream(void)
{
    const leg_info_t *L = &LEG[TEST_LEG];
    con_printf("pot %.1f", (float)hw_millis() / 1000.0f);
    for (int j = 0; j < NJ; j++) {
        int n = L->servo[j];
        con_printf(" %s %.1f (%u/%u)%s", SERVO[n].name, fb_pot[n].deg, (unsigned)fb_pot[n].raw,
                   (unsigned)fb_pot[n].iqr, fb_pot[n].valid ? "" : (fb_trusted(n) ? " INVALID" : " untrusted"));
    }
    con_printf(" vdda %.2f", fb_vdda);
    if (print_all_raw) {
        con_puts(" all");
        for (int n = 0; n < SERVO_COUNT; n++) con_printf(" %u", (unsigned)fb_pot[n].raw);
    }
    con_puts("\r\n");
}

static void banner(void)
{
    const leg_info_t *L = &LEG[TEST_LEG];
    con_printf("\r\n=== leg_lab: %s leg, program %s ===\r\n%s\r\n", L->name, PROGRAM_NAME[PROGRAM], PROGRAM_TEXT[PROGRAM]);
    if (PROGRAM == PROGRAM_MONITOR) con_puts("*** NO PWM: the servo signal pins are held low in this build ***\r\n");
    print_config();
    con_printf("clock: 96 MHz from %s\r\n", hw_clock_from_hsi() ? "HSI (no ST-LINK MCO)" : "ST-LINK 8 MHz MCO");
    con_puts("idle stream: pot <t> <servo> <deg> (<adc>/<spread>) ... vdda <V>\r\n");
    print_help();
}

/* ----------------------------------------------------------- programs --- */

static void run_program(void)
{
    fb_ref_arm(TEST_LEG);
    con_printf("=== start %s: electrical watch on %d reference pots, VDDA %.2f V ===\r\n", PROGRAM_NAME[PROGRAM],
               fb_ref_count(), fb_vdda);
    if (fb_ref_count() == 0) con_puts("warning: no reference pots valid; only VDDA is watched\r\n");
    hw_led(HW_LED_BLUE, false);
    hw_led(HW_LED_RED, false);
    hw_led(HW_LED_GREEN, true);

    mo_begin_program();
    switch (PROGRAM) {
    case PROGRAM_CHECK:  prog_check();  break;
    case PROGRAM_POTCAL: prog_potcal(); break;
    case PROGRAM_LIMITS: prog_limits(); break;
    default: break;
    }
    if (mo_any_on()) mo_park();
    bool cut = mo_was_cut();
    mo_end_program();

    hw_led(HW_LED_GREEN, false);
    hw_led(HW_LED_RED, cut);
    con_printf("=== %s finished: %s. Pulses off. ===\r\n", PROGRAM_NAME[PROGRAM], mo_last_stop());
}

int main(void)
{
    hw_init(TEST_LEG, PROGRAM != PROGRAM_MONITOR);
    fb_update();
    range_reset();
    mo_init();
    fb_ref_arm(TEST_LEG);
    banner();

    uint32_t next_stream = hw_millis(), next_reminder = hw_millis() + 5000u;
    for (;;) {
        mo_tick();
        uint32_t now = hw_millis();
        int  c = mo_char();
        bool pressed = mo_pressed();
        range_update();
        hw_led(HW_LED_BLUE, (now / 500u) & 1u);

        switch (c) {
        case 'r': case 'R': print_ranges(); break;
        case 'c': case 'C': capture_ref(); break;
        case 'a': case 'A': print_all_raw = !print_all_raw; break;
        case 'p': case 'P': print_config(); break;
        case 'h': case 'H': case '?': print_help(); break;
        default: break;
        }

        if (pressed || c == 's' || c == 'S') {
            if (PROGRAM == PROGRAM_MONITOR) {
                con_puts("PROGRAM_MONITOR: nothing to start (no pulses in this build)\r\n");
            } else {
                run_program();
                next_stream = next_reminder = hw_millis() + 5000u;
            }
        }
        if ((int32_t)(now - next_stream) >= 0) {
            next_stream = now + IDLE_STREAM_MS;
            idle_stream();
        }
        if ((int32_t)(now - next_reminder) >= 0) {
            next_reminder = now + 10000u;
            con_printf("idle, last stop: %s\r\n", mo_last_stop());
        }
    }
}
