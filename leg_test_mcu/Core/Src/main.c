/*
 * MOSAC leg test firmware.
 *
 * Moves the three servos of one leg (TEST_LEG in leg_test_config.h) through a fixed test sequence.
 * The leg may start in any position: before any pulse is sent, the potentiometer feedback is read,
 * the PWM is started at that measured angle (so the joint does not jump), and the leg is then ramped
 * slowly to home. The servos of the other legs receive no pulses.
 *
 * Blue button: start / stop. Status is printed on the ST-LINK virtual COM port.
 */
#include "board.h"
#include "feedback.h"
#include "joints.h"
#include "leg_test_config.h"
#include "log.h"
#include "motion.h"
#include "sequence.h"
#include "servo.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

_Static_assert(TEST_LEG >= 0 && TEST_LEG < LEG_COUNT, "TEST_LEG must be LEG_FRONT_RIGHT, LEG_FRONT_LEFT, LEG_BACK_RIGHT or LEG_BACK_LEFT");

typedef enum { ST_IDLE, ST_MEASURE, ST_ENGAGE, ST_HOMING, ST_SEQUENCE, ST_STOPPING, ST_FAULT } state_t;
static const char *const STATE_NAMES[] = { "IDLE", "MEASURE", "ENGAGE", "HOMING", "SEQUENCE", "STOPPING", "FAULT" };

static const uint8_t *leg;        // joint indexes of the leg: body, femur, tibia
static state_t state = ST_IDLE;
static uint32_t state_since;

static float cmd[3];              // commanded servo angles (deg)
static float home[3];             // servo angles of the home pose
static motion_t move;
static bool moving;
static uint8_t waypoint;
static uint32_t hold_since;
static uint32_t loops;

static float measure_min[3], measure_max[3];
static bool measure_in_range[3];
static float engage_deg[3];

static bool track_over[3];
static uint32_t track_over_since[3];

static uint32_t last_log;

static const char *joint_name(uint8_t slot) { return JOINTS[leg[slot]].name; }

// Leg frame (deg from home, policy convention) <-> servo angle
static float leg_to_servo(uint8_t slot, float offset)
{
  joint_type_t type = JOINTS[leg[slot]].type;
  if (offset < JOINT_LIMIT_MIN[type])
    offset = JOINT_LIMIT_MIN[type];
  else if (offset > JOINT_LIMIT_MAX[type])
    offset = JOINT_LIMIT_MAX[type];
  return home[slot] + JOINTS[leg[slot]].sign * offset;
}

static float servo_to_leg(uint8_t slot, float servo_deg)
{
  return (servo_deg - home[slot]) * JOINTS[leg[slot]].sign;
}

static void set_state(state_t s, uint32_t now)
{
  state = s;
  state_since = now;
  last_log = now;
  log_printf("-> %s\r\n", STATE_NAMES[s]);
}

static void apply_cmd(void)
{
  for (uint8_t s = 0; s < 3; s++)
    servo_set_deg(leg[s], cmd[s]);
}

static void release_leg(void)
{
  for (uint8_t s = 0; s < 3; s++)
    servo_release(leg[s]);
  moving = false;
}

static void fault(uint32_t now, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void fault(uint32_t now, const char *fmt, ...)
{
  char reason[200];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(reason, sizeof reason, fmt, ap);
  va_end(ap);

  release_leg();
  set_state(ST_FAULT, now);
  log_printf("FAULT: %s\r\nLeg released (limp). Press the blue button to clear.\r\n", reason);
}

// Debounced press (3 equal samples, 30 ms)
static bool button_event(void)
{
  static bool stable, previous;
  static uint8_t count;
  bool raw = board_button_raw();
  if (raw != stable)
  {
    if (++count >= 3)
    {
      stable = raw;
      count = 0;
    }
  }
  else
  {
    count = 0;
  }
  bool pressed = stable && !previous;
  previous = stable;
  return pressed;
}

static void start_waypoint(uint32_t now)
{
  const waypoint_t *w = &SEQUENCE[waypoint];
  const float offsets[3] = { w->body, w->femur, w->tibia };
  float target[3];
  for (uint8_t s = 0; s < 3; s++)
    target[s] = leg_to_servo(s, offsets[s]);
  motion_plan(&move, cmd, target, SEQUENCE_SPEED_DEG_S, now);
  moving = true;
  log_printf("pose %u/%u \"%s\" (%lu ms)\r\n", (unsigned)(waypoint + 1), (unsigned)SEQUENCE_LENGTH, w->label,
             (unsigned long)move.duration_ms);
}

static void plan_home(uint32_t now)
{
  motion_plan(&move, cmd, home, HOMING_SPEED_DEG_S, now);
  moving = true;
  log_printf("moving to home in %lu ms\r\n", (unsigned long)move.duration_ms);
}

// Releases the leg if a joint cannot follow its command
static bool tracking_ok(uint32_t now)
{
  for (uint8_t s = 0; s < 3; s++)
  {
    float err = fabsf(feedback_deg(s) - cmd[s]);
    if (err <= TRACK_MAX_ERR_DEG)
    {
      track_over[s] = false;
    }
    else if (!track_over[s])
    {
      track_over[s] = true;
      track_over_since[s] = now;
    }
    else if (now - track_over_since[s] >= TRACK_MAX_ERR_MS)
    {
      char e[LOG_DEG_LEN];
      const joint_info_t *jt = &JOINTS[leg[s]];
      fault(now, "%s %s is %s deg away from its command for %lu ms: blocked, no power on header %s, "
                 "or its feedback (%s, %s) is not this servo's",
            JOINT_TYPE_NAMES[s], jt->name, log_deg(e, err), (unsigned long)TRACK_MAX_ERR_MS, jt->pcb_pwm,
            jt->adc_label, jt->pcb_feedback);
      return false;
    }
  }
  return true;
}

static void log_motion(uint32_t now, const char *what)
{
  if (now - last_log < LOG_PERIOD_MS)
    return;
  last_log = now;
  char c[3][LOG_DEG_LEN], m[3][LOG_DEG_LEN];
  for (uint8_t s = 0; s < 3; s++)
  {
    log_deg(c[s], servo_to_leg(s, cmd[s]));
    log_deg(m[s], servo_to_leg(s, feedback_deg(s)));
  }
  log_printf("%-8s %-12s body cmd %6s meas %6s | femur cmd %6s meas %6s | tibia cmd %6s meas %6s\r\n",
             STATE_NAMES[state], what, c[0], m[0], c[1], m[1], c[2], m[2]);
}

static void log_idle(uint32_t now)
{
  if (now - last_log < IDLE_LOG_PERIOD_MS)
    return;
  last_log = now;
  char part[3][48];
  for (uint8_t s = 0; s < 3; s++)
  {
    char d[LOG_DEG_LEN];
    if (feedback_valid(s))
      snprintf(part[s], sizeof part[s], "%s adc %4u = %6s", joint_name(s), feedback_raw(s),
               log_deg(d, servo_to_leg(s, feedback_deg(s))));
    else
      snprintf(part[s], sizeof part[s], "%s adc %4u OUT OF RANGE", joint_name(s), feedback_raw(s));
  }
  log_printf("limp, deg from home: %s | %s | %s\r\n", part[0], part[1], part[2]);
}

static void print_banner(void)
{
  char h[LOG_DEG_LEN];
  log_printf("\r\n=== MOSAC leg test ===\r\n");
  log_printf("Leg under test: %s\r\n", LEG_NAMES[TEST_LEG]);
  log_printf("  joint  type   PWM header  MCU pin  feedback   input  home (servo deg)\r\n");
  for (uint8_t s = 0; s < 3; s++)
  {
    const joint_info_t *jt = &JOINTS[leg[s]];
    log_printf("  %-5s  %-5s  %-10s  %-7s  %-9s  %-5s  %s\r\n", jt->name, JOINT_TYPE_NAMES[s], jt->pcb_pwm,
               jt->pwm_label, jt->pcb_feedback, jt->adc_label, log_deg(h, jt->home_deg));
  }
  for (uint8_t i = 0; i < SEQUENCE_LENGTH; i++)
  {
    const waypoint_t *w = &SEQUENCE[i];
    const float o[3] = { w->body, w->femur, w->tibia };
    for (uint8_t t = 0; t < 3; t++)
      if (o[t] < JOINT_LIMIT_MIN[t] || o[t] > JOINT_LIMIT_MAX[t])
        log_printf("note: pose \"%s\" exceeds the %s limits and will be clipped\r\n", w->label, JOINT_TYPE_NAMES[t]);
  }
  log_printf("The leg is limp: move each joint by hand and check that its own reading changes.\r\n");
  log_printf("Press the blue button to start (and again to stop).\r\n\r\n");
}

static void update_leds(uint32_t now)
{
  bool active = state == ST_MEASURE || state == ST_ENGAGE || state == ST_HOMING || state == ST_SEQUENCE || state == ST_STOPPING;
  board_led(LED_GREEN_PIN, state == ST_IDLE && ((now / 500) & 1));  // slow blink: limp, waiting
  board_led(LED_BLUE_PIN, active && (state == ST_SEQUENCE || ((now / 125) & 1)));  // fast blink: moving to/from home
  board_led(LED_RED_PIN, state == ST_FAULT);
}

static void check_measurement(uint32_t now)
{
  for (uint8_t s = 0; s < 3; s++)
  {
    const joint_info_t *jt = &JOINTS[leg[s]];
    char a[LOG_DEG_LEN], b[LOG_DEG_LEN];
    if (!measure_in_range[s])
    {
      fault(now, "%s %s feedback reads %u counts, outside its calibration: check the wire on %s (%s)",
            JOINT_TYPE_NAMES[s], jt->name, feedback_raw(s), jt->pcb_feedback, jt->adc_label);
      return;
    }
    if (measure_max[s] - measure_min[s] > MEASURE_MAX_SPREAD_DEG)
    {
      fault(now, "%s %s feedback is not steady (%s deg spread): loose or unplugged wire on %s (%s), or the leg moved",
            JOINT_TYPE_NAMES[s], jt->name, log_deg(a, measure_max[s] - measure_min[s]), jt->pcb_feedback, jt->adc_label);
      return;
    }
    float from_home = servo_to_leg(s, feedback_deg(s));
    if (fabsf(from_home) > START_MAX_FROM_HOME_DEG)
    {
      fault(now, "%s %s reads %s deg from home (limit %s): feedback wiring or calibration problem",
            JOINT_TYPE_NAMES[s], jt->name, log_deg(a, from_home), log_deg(b, START_MAX_FROM_HOME_DEG));
      return;
    }
  }

  // Start the pulses exactly where each servo already is
  char d[3][LOG_DEG_LEN];
  for (uint8_t s = 0; s < 3; s++)
  {
    cmd[s] = engage_deg[s] = feedback_deg(s);
    log_deg(d[s], servo_to_leg(s, cmd[s]));
  }
  apply_cmd();
  set_state(ST_ENGAGE, now);
  log_printf("engaged at the measured position (deg from home): body %s, femur %s, tibia %s\r\n", d[0], d[1], d[2]);
}

static void control_step(uint32_t now)
{
  feedback_update();
  bool pressed = button_event();

  switch (state)
  {
    case ST_IDLE:
      if (pressed)
      {
        for (uint8_t s = 0; s < 3; s++)
        {
          measure_min[s] = measure_max[s] = feedback_deg(s);
          measure_in_range[s] = true;
          track_over[s] = false;
        }
        set_state(ST_MEASURE, now);
        break;
      }
      log_idle(now);
      break;

    case ST_MEASURE:
      if (pressed)
      {
        set_state(ST_IDLE, now);
        break;
      }
      for (uint8_t s = 0; s < 3; s++)
      {
        float d = feedback_deg(s);
        if (!feedback_valid(s))
          measure_in_range[s] = false;
        if (d < measure_min[s]) measure_min[s] = d;
        if (d > measure_max[s]) measure_max[s] = d;
      }
      if (now - state_since >= MEASURE_TIME_MS)
        check_measurement(now);
      break;

    case ST_ENGAGE:
      if (pressed)
      {
        release_leg();
        set_state(ST_IDLE, now);
        break;
      }
      for (uint8_t s = 0; s < 3; s++)
      {
        float jump = fabsf(feedback_deg(s) - engage_deg[s]);
        if (jump > ENGAGE_MAX_JUMP_DEG)
        {
          char j[LOG_DEG_LEN];
          const joint_info_t *jt = &JOINTS[leg[s]];
          fault(now, "%s %s moved %s deg when its pulses started: its feedback (%s, %s) is probably another servo's, "
                     "or this servo is not on header %s",
                JOINT_TYPE_NAMES[s], jt->name, log_deg(j, jump), jt->adc_label, jt->pcb_feedback, jt->pcb_pwm);
          return;
        }
      }
      if (now - state_since >= ENGAGE_SETTLE_MS)
      {
        set_state(ST_HOMING, now);
        plan_home(now);
      }
      break;

    case ST_HOMING:
      if (pressed)
      {
        // Already heading home: finish the move, then release
        set_state(ST_STOPPING, now);
        break;
      }
      if (!tracking_ok(now))
        break;
      if (motion_sample(&move, now, cmd))
      {
        apply_cmd();
        waypoint = 0;
        loops = 0;
        set_state(ST_SEQUENCE, now);
        start_waypoint(now);
        break;
      }
      apply_cmd();
      log_motion(now, "to home");
      break;

    case ST_SEQUENCE:
      if (pressed)
      {
        set_state(ST_STOPPING, now);
        plan_home(now);
        break;
      }
      if (!tracking_ok(now))
        break;
      if (moving)
      {
        if (motion_sample(&move, now, cmd))
        {
          moving = false;
          hold_since = now;
        }
        apply_cmd();
      }
      else if (now - hold_since >= SEQUENCE[waypoint].hold_ms)
      {
        if (++waypoint >= SEQUENCE_LENGTH)
        {
          waypoint = 0;
          log_printf("sequence done %lu time(s)\r\n", (unsigned long)++loops);
        }
        start_waypoint(now);
      }
      log_motion(now, SEQUENCE[waypoint].label);
      break;

    case ST_STOPPING:
      if (!tracking_ok(now))
        break;
      if (motion_sample(&move, now, cmd))
      {
        apply_cmd();
        release_leg();
        set_state(ST_IDLE, now);
        log_printf("back home, leg released (limp)\r\n");
        break;
      }
      apply_cmd();
      log_motion(now, "to home");
      break;

    case ST_FAULT:
      if (pressed)
        set_state(ST_IDLE, now);
      break;
  }

  update_leds(now);
}

int main(void)
{
  leg = LEG_JOINTS[TEST_LEG];

  board_init(leg);
  feedback_init(leg);
  for (uint8_t s = 0; s < 3; s++)
    home[s] = JOINTS[leg[s]].home_deg;

  HAL_Delay(50);  // let the feedback buffer fill once
  print_banner();

  uint32_t next = HAL_GetTick();
  state_since = next;
  while (1)
  {
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - next) >= 0)
    {
      next += CONTROL_PERIOD_MS;
      if ((int32_t)(now - next) > 10 * CONTROL_PERIOD_MS)
        next = now + CONTROL_PERIOD_MS;  // fell behind: resynchronize instead of bursting
      control_step(now);
    }
  }
}
