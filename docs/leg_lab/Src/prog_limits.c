/* PROGRAM_LIMITS: finds where each joint of the leg is stopped by the structure,
 * and how the femur changes the tibia and body ranges. Robot lifted, foot free.
 *
 *   0. every pot valid and trusted; the tibia needs its hand range or its
 *      calibrator pose (the linkage can fold past a toggle without any contact,
 *      so contact detection alone cannot protect it)
 *   1. all three joints switched on where they are (probe, settle, wiggle); they
 *      hold that shape
 *   2. a window per joint: pulse range, travel cap, hand range + 3 deg, tibia
 *      linkage model. Nothing outside it is ever commanded
 *   3. 1-D limits, body, femur, tibia, high side then low side: guarded search at
 *      4 deg/s (1 deg/s from the first sign of contact), stop pushing, back off
 *      4 deg, approach again at 1.5 deg/s (repeatability), back to the start
 *   4. femur x tibia map: the femur at steps of MAP_FEMUR_STEP_DEG inside its
 *      limits; at each, the tibia limits (collisions depend on both)
 *   5. femur x body map: body limits with the femur near each end
 *   6. back to the rest pose, pulses off
 * Results: human-readable lines and RES,... lines for tools/leg_lab_log.py. */
#include "config.h"
#include "console.h"
#include "feedback.h"
#include "guard.h"
#include "motion.h"
#include "programs.h"
#include "robot.h"
#include <string.h>

typedef struct {
    bool          found;
    search_kind_t kind;
    float         lim;      /* command deg */
    float         repeat;   /* |second - first| approach, < 0 if not refined */
    int           other;
} side_t;

static const float TRAVEL[NJ] = LIMIT_TRAVEL_DEG;
static float start_cmd[NJ];

static float absf(float v) { return v < 0.0f ? -v : v; }

static const char *side_name(int dir) { return dir > 0 ? "high" : "low"; }

static void print_result(int j, int dir, const side_t *s, const char *ctx)
{
    int   n = mo_j[j].servo;
    float pot = s->lim + mo_j[j].offset;
    float fem = mo_j[JOINT_FEMUR].out;
    con_printf("  %s %s: %.1f deg (%u us), %s", SERVO[n].name, side_name(dir), s->lim, (unsigned)mo_us(s->lim),
               SEARCH_KIND_NAME[s->kind]);
    if (s->kind == S_CONTACT_OTHER && s->other >= 0) con_printf(" (%s was pushed)", SERVO[mo_j[s->other].servo].name);
    if (s->kind == S_CAP) con_printf(" (%s)", dir > 0 ? "window high edge" : "window low edge");
    if (s->repeat >= 0.0f) con_printf(", repeat %.1f", s->repeat);
    if (guard_has_ref(n)) con_printf(", q %.1f", guard_q_of_pot(n, pot));
    con_puts("\r\n");
    /* RES,limit,<leg>,<servo>,<context>,femur,<femur cmd>,<side>,<kind>,cmd,..,us,..,pot,..,repeat,..[,q,..] */
    con_printf("RES,limit,%s,%s,%s,femur,%.2f,%s,%s,cmd,%.2f,us,%u,pot,%.2f,repeat,%.2f", LEG[TEST_LEG].tag,
               SERVO[n].name, ctx, fem, side_name(dir), SEARCH_KIND_NAME[s->kind], s->lim, (unsigned)mo_us(s->lim),
               pot, s->repeat);
    if (guard_has_ref(n)) con_printf(",q,%.2f", guard_q_of_pot(n, pot));
    con_puts("\r\n");
}

/* One side of one joint: search, back off, optional refine, back off. The joint
 * is left backed off from the limit (not at the start). */
static result_t find_side(int j, int dir, bool refine, side_t *out)
{
    memset(out, 0, sizeof *out);
    out->repeat = -1.0f;
    out->other  = -1;

    search_t s = mo_search(j, dir, SEARCH_SPEED_DEG_S, SEARCH_DETECT_DEG);
    if (s.kind == S_ELECTRICAL) {
        /* Either a stall (current) or an outside disturbance: wait for it to clear
         * and try once more. A real stall shows again at the same place. */
        con_printf("  %s %s: electrical sign at %.1f deg, trying once more\r\n", SERVO[mo_j[j].servo].name,
                   side_name(dir), s.onset);
        result_t r0 = mo_move_joint(j, s.at_end - (float)dir * SEARCH_BACKOFF_DEG, MAX_SPEED_DEG_S, 800);
        if (r0 != R_OK) return r0;
        s = mo_search(j, dir, SEARCH_SPEED_DEG_S, SEARCH_DETECT_DEG);
    }
    if (s.kind == S_CUT)     return R_CUT;
    if (s.kind == S_STOPPED) return R_STOPPED;
    out->found = true;
    out->kind  = s.kind;
    out->lim   = s.onset;
    out->other = s.other;
    if (s.kind == S_CAP) return R_OK;

    result_t r = mo_move_joint(j, s.at_end - (float)dir * SEARCH_BACKOFF_DEG, MAX_SPEED_DEG_S, 300);
    if (r != R_OK) return r;
    if (refine && s.kind == S_CONTACT) {
        search_t s2 = mo_search(j, dir, REFINE_SPEED_DEG_S, REFINE_DETECT_DEG);
        if (s2.kind == S_CUT)     return R_CUT;
        if (s2.kind == S_STOPPED) return R_STOPPED;
        if (s2.kind == S_CONTACT) {
            out->repeat = absf(s2.onset - s.onset);
            out->lim    = s2.onset;
        }
        r = mo_move_joint(j, s2.at_end - (float)dir * SEARCH_BACKOFF_DEG, MAX_SPEED_DEG_S, 300);
    }
    return r;
}

static result_t joint_limits(int j, side_t res[2], const char *ctx, bool refine)
{
    result_t r = R_OK;
    float from = mo_j[j].cmd;
    for (int side = 1; side >= 0 && r == R_OK; side--) {
        int dir = side ? +1 : -1;
        r = find_side(j, dir, refine, &res[side]);
        if (res[side].found) print_result(j, dir, &res[side], ctx);
        if (r == R_OK) r = mo_move_joint(j, from, MAX_SPEED_DEG_S, 200);
    }
    return r;
}

static bool usable(const side_t *s) { return s->found; }

/* femur x (tibia | body) map: the femur goes to each of `points` (with the other
 * joint in the middle of its last known range), then the other joint's limits
 * are searched there. A femur move that is blocked ends the map on that side. */
static result_t map_with_femur(int other, const float *points, int count, const char *what)
{
    result_t r = R_OK;
    float park_other = start_cmd[other];
    for (int i = 0; i < count && r == R_OK; i++) {
        float g = points[i];
        con_printf("--- %s map %d/%d: femur %s -> %.1f deg ---\r\n", what, i + 1, count,
                   SERVO[mo_j[JOINT_FEMUR].servo].name, g);
        if ((r = mo_move_joint(other, park_other, MAX_SPEED_DEG_S, 200)) != R_OK) break;
        r = mo_move_joint(JOINT_FEMUR, g, MAX_SPEED_DEG_S, 300);
        if (r == R_BLOCKED) {
            con_printf("  femur blocked on the way to %.1f with %s at %.1f: the map ends on this side\r\n", g,
                       SERVO[mo_j[other].servo].name, park_other);
            r = R_OK;
            break;
        }
        if (r != R_OK) break;
        side_t res[2];
        r = joint_limits(other, res, "map", false);
        if (r != R_OK) break;
        if (usable(&res[0]) && usable(&res[1])) park_other = 0.5f * (res[0].lim + res[1].lim);
        con_printf("RES,map,%s,%s,femur,%.2f,low,%s,%.2f,high,%s,%.2f\r\n", LEG[TEST_LEG].tag,
                   SERVO[mo_j[other].servo].name, mo_j[JOINT_FEMUR].out, SEARCH_KIND_NAME[res[0].kind], res[0].lim,
                   SEARCH_KIND_NAME[res[1].kind], res[1].lim);
    }
    /* Back the way it came: femur first (the other joint is where it was fine at
     * the last point), then the other joint. */
    if (r == R_OK) r = mo_move_joint(JOINT_FEMUR, start_cmd[JOINT_FEMUR], MAX_SPEED_DEG_S, 200);
    if (r == R_OK) r = mo_move_joint(other, start_cmd[other], MAX_SPEED_DEG_S, 200);
    return r;
}

void prog_limits(void)
{
    static side_t res[NJ][2];
    memset(res, 0, sizeof res);
    con_printf("=== limit search, %s leg ===\r\n", LEG[TEST_LEG].name);
    con_puts("The robot must be lifted with this foot touching nothing. USER or 'x' eases back and stops;\r\n"
             "pressing again (or 'k') cuts the pulses at once.\r\n");

    /* 0. preconditions */
    for (int j = 0; j < NJ; j++) {
        int n = mo_j[j].servo;
        if (!fb_trusted(n) || !fb_pot[n].valid) {
            con_printf("Refused: %s pot is %s (adc %u, spread %u). All three joints hold the leg's shape, so all\r\n"
                       "three need a valid, trusted pot.\r\n",
                       SERVO[n].name, fb_trusted(n) ? "invalid" : "UNTRUSTED", (unsigned)fb_pot[n].raw,
                       (unsigned)fb_pot[n].iqr);
            mo_note_stop("limit search refused: pot not usable", n);
            return;
        }
    }
    {
        int n = mo_j[JOINT_TIBIA].servo;
        if ((LIMITS_JOINTS & (1u << JOINT_TIBIA)) && !guard_has_hand(n) && !guard_has_ref(n)) {
            con_printf("Refused: %s has neither a hand range nor a calibrator pose in config.h. The tibia four-bar\r\n"
                       "can be driven through a toggle without any contact, so the search needs one of them.\r\n"
                       "Use PROGRAM_MONITOR: 'r' after moving the leg by hand, or 'c' with the calibrator on.\r\n",
                       SERVO[n].name);
            mo_note_stop("limit search refused: tibia has no hand range or calibrator pose", n);
            return;
        }
    }
    for (int j = 0; j < NJ; j++) {
        int n = mo_j[j].servo;
        con_printf("RES,meta,%s,%s,joint,%s,dir,%d,ref,%.2f,hand,%.2f,%.2f\r\n", LEG[TEST_LEG].tag, SERVO[n].name,
                   JOINT_NAME[j], SERVO[n].dir, guard_ref(n), guard_hand(n, 0), guard_hand(n, 1));
    }

    /* 1. switch on, one at a time */
    con_puts("switch-on, one joint at a time:\r\n");
    for (int j = 0; j < NJ; j++)
        if (mo_switch_on(j) != R_OK) return;

    /* 2. windows */
    con_puts("windows (command deg):\r\n");
    for (int j = 0; j < NJ; j++) {
        int n = mo_j[j].servo;
        start_cmd[j] = mo_j[j].out;
        window_t w = guard_window(n, j, start_cmd[j], TRAVEL[j], mo_j[j].offset);
        if (start_cmd[j] < w.lo || start_cmd[j] > w.hi)
            con_printf("  %s starts at %.1f, outside its window: the window is widened to include it\r\n",
                       SERVO[n].name, start_cmd[j]);
        mo_set_window(j, w.lo, w.hi);
        guard_print(n, &w, mo_j[j].offset);
        con_printf("RES,window,%s,%s,start,%.2f,offset,%.2f,lo,%.2f,%s,hi,%.2f,%s\r\n", LEG[TEST_LEG].tag,
                   SERVO[n].name, start_cmd[j], mo_j[j].offset, mo_j[j].lo, w.lo_src, mo_j[j].hi, w.hi_src);
    }

    /* 3. 1-D limits */
    result_t r = R_OK;
    for (int j = 0; j < NJ && r == R_OK; j++) {
        if (!(LIMITS_JOINTS & (1u << j))) continue;
        con_printf("--- %s (%s): limits from %.1f deg ---\r\n", SERVO[mo_j[j].servo].name, JOINT_NAME[j], start_cmd[j]);
        r = joint_limits(j, res[j], "start", LIMITS_REFINE != 0);
    }

    /* 4. femur x tibia map */
    const bool map_ft = LIMITS_MAP_FT != 0, map_fb = LIMITS_MAP_FB != 0;
    const bool do_tibia = (LIMITS_JOINTS & (1u << JOINT_TIBIA)) != 0u, do_body = (LIMITS_JOINTS & (1u << JOINT_BODY)) != 0u;
    const side_t *fem = res[JOINT_FEMUR];
    bool fem_ok = usable(&fem[0]) && usable(&fem[1]);
    float f_lo = fem[0].lim + MAP_MARGIN_DEG, f_hi = fem[1].lim - MAP_MARGIN_DEG;
    if (r == R_OK && map_ft && fem_ok && f_hi > f_lo && do_tibia) {
        float pts[32];
        int   cnt = 0;
        /* outwards from the start: up to f_hi, then down to f_lo */
        for (float g = start_cmd[JOINT_FEMUR] + MAP_FEMUR_STEP_DEG; g <= f_hi && cnt < 16; g += MAP_FEMUR_STEP_DEG) pts[cnt++] = g;
        if (cnt == 0 || pts[cnt - 1] < f_hi - 1.0f) { if (cnt < 16) pts[cnt++] = f_hi; }
        int up_cnt = cnt;
        r = map_with_femur(JOINT_TIBIA, pts, up_cnt, "tibia");
        cnt = 0;
        for (float g = start_cmd[JOINT_FEMUR] - MAP_FEMUR_STEP_DEG; g >= f_lo && cnt < 16; g -= MAP_FEMUR_STEP_DEG) pts[cnt++] = g;
        if (cnt == 0 || pts[cnt - 1] > f_lo + 1.0f) { if (cnt < 16) pts[cnt++] = f_lo; }
        if (r == R_OK) r = map_with_femur(JOINT_TIBIA, pts, cnt, "tibia");
    } else if (map_ft && r == R_OK) {
        con_puts("femur x tibia map skipped (needs both femur limits and the tibia search)\r\n");
    }

    /* 5. femur x body map */
    if (r == R_OK && map_fb && fem_ok && f_hi > f_lo && do_body) {
        float pts[2] = { f_hi, f_lo };
        r = map_with_femur(JOINT_BODY, pts, 1, "body");
        if (r == R_OK) r = map_with_femur(JOINT_BODY, pts + 1, 1, "body");
    } else if (map_fb && r == R_OK) {
        con_puts("femur x body map skipped (needs both femur limits and the body search)\r\n");
    }

    /* summary */
    con_puts("=== limits found (command deg, pulse us) ===\r\n");
    for (int j = 0; j < NJ; j++) {
        if (!(LIMITS_JOINTS & (1u << j))) continue;
        int n = mo_j[j].servo;
        con_printf("  %s  start %.1f", SERVO[n].name, start_cmd[j]);
        for (int side = 0; side < 2; side++) {
            const side_t *s = &res[j][side];
            if (!s->found) { con_printf("  %s -", side_name(side ? 1 : -1)); continue; }
            con_printf("  %s %.1f (%u us, %s)", side_name(side ? 1 : -1), s->lim, (unsigned)mo_us(s->lim),
                       SEARCH_KIND_NAME[s->kind]);
        }
        if (res[j][0].found && res[j][1].found) con_printf("  range %.1f", res[j][1].lim - res[j][0].lim);
        con_puts("\r\n");
    }
    if (r == R_OK)       mo_note_stop("limit search done", -1);
    else if (r == R_STOPPED) mo_note_stop("limit search stopped by user", -1);
    else if (r == R_BLOCKED) mo_note_stop("limit search: a return move was blocked", -1);
}
