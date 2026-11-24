/**
 * servo_test_leg: stand-alone servo test firmware for the MOSAC quadruped.
 * Target: NUCLEO-F412ZG (STM32F412ZG).
 *
 * Moves the selected legs slowly through a fixed sequence of joint sweeps,
 * open loop, and reads each servo's pot feedback on ADC1. The feedback is
 * converted to degrees with quadruped_control_mcu/Core/Inc/calibration_ADC.h.
 * It is used to start without jumps and to stop on faults, not to control the
 * servos. Servos of legs that are not selected get no PWM signal.
 *
 * Motion safety
 *   - Every planned move is eased and speed limited (MAX_SPEED_DEG_S peak).
 *   - A hard slew limit (MAX_SLEW_DEG_S) applies to every command sent, so a
 *     pulse width can never jump.
 *   - Start: servos switch on one at a time, each at the angle its pot reads,
 *     so the first pulse hardly moves it. The pot offset is learned during a
 *     short hold. A servo that does not follow its first command aborts the
 *     start. Then all joints ease slowly to neutral.
 *   - Fault cut-out: if a joint's averaged error goes above FAULT_ERR_DEG
 *     (blocked, stalled or oscillating), all pulses are cut.
 *
 *   reset / power-up  No servo pulses. Blue LED blinks. Feedback is streamed,
 *                     so joints can be moved by hand to check the pots.
 *   USER or 's'       Start as above, then loop the sequence. Green LED on.
 *   USER or 'x'       Ease slowly back to neutral, then pulses stop.
 *   'a'               Also print the raw ADC value of all 12 feedback channels.
 *   fault             All pulses stop. Red LED on.
 *
 * Serial: ST-LINK virtual COM port (USART3, 115200 8N1).
 */
#include "stm32f4xx_hal.h"
#include "calibration_ADC.h"   /* from quadruped_control_mcu/Core/Inc (CALIB_DIR) */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define LEG_FR    0
#define LEG_FL    1
#define LEG_BR    2
#define LEG_BL    3
#define LEG_COUNT 4
#define LEG_BIT(l) (1u << (l))
#define ALL_LEGS  (LEG_BIT(LEG_FR) | LEG_BIT(LEG_FL) | LEG_BIT(LEG_BR) | LEG_BIT(LEG_BL))

#define MODE_TOGETHER   0   /* every selected leg runs the sequence at the same time */
#define MODE_ONE_BY_ONE 1   /* FR, FL, BR, BL in turn; the others hold neutral       */

/* ============================ Configuration ============================ */

#define PROGRAM_SEQUENCE    0       /* slow test sequence around neutral (verified legs)    */
#define PROGRAM_CHECK       1       /* troubleshooting: one servo at a time, small moves
                                       around where it is now; no neutral needed          */
#define PROGRAM_LIMITS      2       /* find each joint's mechanical limits, slowly        */

#define PROGRAM             PROGRAM_LIMITS
#define MOTION_ENABLED      1       /* 0 = feedback only: no servo pulses can be sent */
#define TEST_LEGS           LEG_BIT(LEG_FR) /* bit mask, ALL_LEGS for all four        */
#define TEST_MODE           MODE_TOGETHER   /* MODE_TOGETHER or MODE_ONE_BY_ONE       */
#define SEQUENCE_REPEATS    0       /* rounds before parking, 0 = until stopped       */

#define MAX_SPEED_DEG_S     10.0f   /* peak joint speed of every planned move         */
#define MAX_SLEW_DEG_S      15.0f   /* hard limit on how fast any command can change  */
#define MIN_MOVE_MS         400u    /* shortest planned move                          */
#define SETTLE_MS           400u    /* start: hold each servo at its measured angle   */
#define MAX_OFFSET_DEG      5.0f    /* start aborts if a servo ends further off       */
#define FAULT_ERR_DEG       4.0f    /* averaged |error| above this cuts all pulses    */
#define FAULT_TAU_MS        100.0f  /* averaging time constant of that error          */
#define PARK_HOLD_MS        300u    /* hold neutral before cutting the pulses         */
#define TELEMETRY_MS        100u    /* feedback print period                          */
#define FB_MAX_IQR          40u     /* pot trusted only if its ADC spread is below this */

/* PROGRAM_CHECK settings */
#define CHECK_SPEED_DEG_S   5.0f    /* peak speed of the check sweeps                 */
#define CHECK_PROBE_MS      300u    /* first pulse: watch the pots this long at 1 ms  */
#define CHECK_PROBE_CUT_DEG 4.0f    /* any pot of the leg more than this off for ...   */
#define CHECK_PROBE_SUSTAIN_MS 20u  /* ... this long on the first pulse cuts the servo  */
#define CHECK_PROBE_FAST_DEG 10.0f  /* or more than this at any instant. Brief dips of a
                                       few degrees when a motor starts are electrical
                                       (all pots shift together), not movement.         */
#define CHECK_HOLD_MS       400u    /* hold at each sweep point before reading it     */
#define CHECK_JOINTS        7u      /* joints to check: bits (1 << JOINT_BODY/FEMUR/TIBIA), 7 = all */
#define CHECK_GAIN_MIN      0.8f    /* pot change / commanded change must be in range */
#define CHECK_GAIN_MAX      1.2f
static const float CHECK_AMPLITUDE_DEG[] = { 5.0f, 10.0f };   /* sweeps: +A, -A, back */

/* PROGRAM_LIMITS settings */
#define LIMIT_JOINTS        7u      /* joints to search: bits (1 << JOINT_BODY/FEMUR/TIBIA) */
#define LIMIT_SPEED_DEG_S   5.0f    /* constant search speed towards a limit          */
#define LIMIT_LAG_DEG       3.0f    /* pot this far behind the command ...            */
#define LIMIT_DETECT_MS     150u    /* ... for this long = limit (blocked)            */
#define LIMIT_BACKOFF_DEG   5.0f    /* back off this far from a limit                 */
#define LIMIT_MAX_TRAVEL_DEG 135.0f /* never search further than this from the start  */
#define LIMIT_MIN_DEG       5.0f    /* absolute command range of the search           */
#define LIMIT_MAX_DEG       260.0f

/* Servo PWM timing, same as quadruped_control_mcu (1 us timer tick). */
#define PWM_PERIOD_US       3100u   /* ~322 Hz frame (digital servos)  */
#define US_PER_DEG          (1000.0f / 140.0f)
#define US_AT_0_DEG         500.0f
#define PULSE_MIN_US        500.0f
#define PULSE_MAX_US        2500.0f
#define UPDATE_PERIOD_MS    10u     /* control loop period (100 Hz)    */

/* Feedback: ADC1 scans all 12 pot channels continuously into a DMA ring of
 * ADC_SCANS scans (480-cycle samples, ~0.25 ms per scan, ~16 ms window).
 * Each channel is reported as the median of the window. */
#define ADC_SCANS           64

_Static_assert(TEST_LEGS != 0 && (TEST_LEGS & ~ALL_LEGS) == 0, "TEST_LEGS must be a non-empty mask of LEG_BIT(...)");
_Static_assert(TEST_MODE == MODE_TOGETHER || TEST_MODE == MODE_ONE_BY_ONE, "invalid TEST_MODE");
_Static_assert(MAX_SLEW_DEG_S > MAX_SPEED_DEG_S, "the slew limit must not slow down planned moves");

/* Joints of one leg, and the allowed offset from neutral (degrees).
 * Limits are the BODY/FEMUR/TIBIA_RANGE values from quadruped_control_mcu/Core/Inc/main.h. */
enum { JOINT_BODY, JOINT_FEMUR, JOINT_TIBIA, JOINTS_PER_LEG };

static const float JOINT_LIMIT_DEG[JOINTS_PER_LEG][2] = {
    [JOINT_BODY]  = { -10.0f, 15.0f },
    [JOINT_FEMUR] = { -20.0f, 30.0f },
    [JOINT_TIBIA] = { -15.0f, 15.0f },
};

/* The predefined movement, applied to every moving leg. Each keyframe is a pose
 * in degrees from neutral. The leg moves there at MAX_SPEED_DEG_S (the time is
 * worked out from the largest joint move), then holds for hold_ms. The
 * feedback is logged as "settled" at the end of every hold. Values outside
 * JOINT_LIMIT_DEG are clamped. */
typedef struct {
    float    deg[JOINTS_PER_LEG];   /* body, femur, tibia */
    uint16_t hold_ms;
} keyframe_t;

static const keyframe_t SEQUENCE[] = {
    /*   body femur tibia    hold */
    { {   0,    0,    0 }, 1000 },  /* settle at neutral          */

    { {  10,    0,    0 },  500 },  /* 1. body (hip) joint alone  */
    { {  -8,    0,    0 },  500 },
    { {   0,    0,    0 },  500 },

    { {   0,   20,    0 },  500 },  /* 2. femur alone             */
    { {   0,  -15,    0 },  500 },
    { {   0,    0,    0 },  500 },

    { {   0,    0,   12 },  500 },  /* 3. tibia alone             */
    { {   0,    0,  -12 },  500 },
    { {   0,    0,    0 },  500 },

    { {   5,   15,  -10 },    0 },  /* 4. all three together, x2  */
    { {  -5,   15,   10 },    0 },
    { {  -5,  -10,   10 },    0 },
    { {   5,  -10,  -10 },    0 },
    { {   5,   15,  -10 },    0 },
    { {  -5,   15,   10 },    0 },
    { {  -5,  -10,   10 },    0 },
    { {   5,  -10,  -10 },    0 },

    { {   0,    0,    0 }, 1500 },  /* back to neutral, pause     */
};
#define SEQUENCE_LEN (sizeof SEQUENCE / sizeof SEQUENCE[0])

/* ========================== Robot description ========================== */

/* Servo numbering used across this repo (calibration_ADC.h, quadruped_control_mcu):
 * servo n is driven by TIM2/TIM3/TIM4 for n/4, channel n%4 + 1, and its pot is
 * ADC1 rank n+1 (same channel order as quadruped_control_mcu and Programmed_Gait).
 * neutral_deg: open-loop neutral angle ("WITHOUT PID" mid points, Programmed_Gait).
 * dir: mirrored mounting, matches the joint_range table in quadruped_control_mcu. */
typedef struct {
    const char   *name;
    const char   *pin_name;
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       af;
    float         neutral_deg;
    int8_t        dir;
    uint32_t      adc_ch;      /* ADC_CHANNEL_n has the value n */
    const char   *fb_pin;
} servo_t;

static const servo_t SERVOS[12] = {
    /*  n      name   PWM pin                                  neutral dir  feedback          */
    /*  0 */ { "BFR", "PA15", GPIOA, GPIO_PIN_15, GPIO_AF1_TIM2, 94.0f, +1, ADC_CHANNEL_4,  "PA4" },
    /*  1 */ { "BBR", "PA1",  GPIOA, GPIO_PIN_1,  GPIO_AF1_TIM2, 87.0f, -1, ADC_CHANNEL_13, "PC3" },
    /*  2 */ { "BBL", "PB10", GPIOB, GPIO_PIN_10, GPIO_AF1_TIM2, 86.0f, -1, ADC_CHANNEL_2,  "PA2" },
    /*  3 */ { "TFR", "PB11", GPIOB, GPIO_PIN_11, GPIO_AF1_TIM2, 89.0f, +1, ADC_CHANNEL_11, "PC1" },
    /*  4 */ { "FFR", "PC6",  GPIOC, GPIO_PIN_6,  GPIO_AF2_TIM3, 87.0f, +1, ADC_CHANNEL_14, "PC4" },
    /*  5 */ { "BFL", "PC7",  GPIOC, GPIO_PIN_7,  GPIO_AF2_TIM3, 94.0f, +1, ADC_CHANNEL_7,  "PA7" },
    /*  6 */ { "FFL", "PC8",  GPIOC, GPIO_PIN_8,  GPIO_AF2_TIM3, 87.0f, -1, ADC_CHANNEL_5,  "PA5" },
    /*  7 */ { "TFL", "PC9",  GPIOC, GPIO_PIN_9,  GPIO_AF2_TIM3, 93.0f, -1, ADC_CHANNEL_6,  "PA6" },
    /*  8 */ { "FBL", "PD12", GPIOD, GPIO_PIN_12, GPIO_AF2_TIM4, 94.0f, -1, ADC_CHANNEL_10, "PC0" },
    /*  9 */ { "TBL", "PD13", GPIOD, GPIO_PIN_13, GPIO_AF2_TIM4, 96.0f, -1, ADC_CHANNEL_3,  "PA3" },
    /* 10 */ { "TBR", "PD14", GPIOD, GPIO_PIN_14, GPIO_AF2_TIM4, 80.0f, +1, ADC_CHANNEL_12, "PC2" },
    /* 11 */ { "FBR", "PD15", GPIOD, GPIO_PIN_15, GPIO_AF2_TIM4, 85.0f, +1, ADC_CHANNEL_9,  "PB1" },
};

/* Pot calibration of servo n: ADC sums of MEAS_PER_ANG readings at ANGLES[], [0] going down, [1] going up. */
static const float (*const CALIB_TABLE[12])[MED_CALIB] = {
    ADC_VALUES_SERVO_0, ADC_VALUES_SERVO_1, ADC_VALUES_SERVO_2,  ADC_VALUES_SERVO_3,
    ADC_VALUES_SERVO_4, ADC_VALUES_SERVO_5, ADC_VALUES_SERVO_6,  ADC_VALUES_SERVO_7,
    ADC_VALUES_SERVO_8, ADC_VALUES_SERVO_9, ADC_VALUES_SERVO_10, ADC_VALUES_SERVO_11,
};

typedef struct {
    const char *name;
    const char *tag;
    uint8_t     servo[JOINTS_PER_LEG];   /* body, femur, tibia */
} leg_t;

static const leg_t LEGS[LEG_COUNT] = {
    [LEG_FR] = { "front right", "FR", { 0, 4, 3 } },    /* BFR, FFR, TFR */
    [LEG_FL] = { "front left",  "FL", { 5, 6, 7 } },    /* BFL, FFL, TFL */
    [LEG_BR] = { "back right",  "BR", { 1, 11, 10 } },  /* BBR, FBR, TBR */
    [LEG_BL] = { "back left",   "BL", { 2, 8, 9 } },    /* BBL, FBL, TBL */
};

static const char *const JOINT_NAME[JOINTS_PER_LEG] = { "body ", "femur", "tibia" };

/* Nucleo-144 LEDs and USER button */
#define LED_GREEN  GPIO_PIN_0    /* LD1, PB0  */
#define LED_BLUE   GPIO_PIN_7    /* LD2, PB7  */
#define LED_RED    GPIO_PIN_14   /* LD3, PB14 */
#define USER_BTN_PORT GPIOC
#define USER_BTN_PIN  GPIO_PIN_13 /* B1, high when pressed */

/* ================================ State ================================ */

static TIM_HandleTypeDef  htim[3];          /* TIM2, TIM3, TIM4 */
static UART_HandleTypeDef huart3;
static ADC_HandleTypeDef  hadc1;
static DMA_HandleTypeDef  hdma_adc1;
static const uint32_t     TIM_CH[4] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };
static bool               clock_from_hsi;

/* Per joint, indexed [leg][joint]. All angles in degrees on the PWM scale. */
static bool  joint_on[LEG_COUNT][JOINTS_PER_LEG];      /* pulses enabled                   */
static float cmd_deg[LEG_COUNT][JOINTS_PER_LEG];       /* planned command                  */
static float out_deg[LEG_COUNT][JOINTS_PER_LEG];       /* command sent (slew limited)      */
static float offset_deg[LEG_COUNT][JOINTS_PER_LEG];    /* learned pot offset (meas - sent) */
static bool  offset_known[LEG_COUNT][JOINTS_PER_LEG];
static float err_avg[LEG_COUNT][JOINTS_PER_LEG];       /* averaged |error| for the cut-out */

static volatile uint16_t adc_buf[ADC_SCANS][12];   /* DMA ring, [scan][servo]          */
static uint16_t fb_raw[12];                        /* median ADC code                  */
static uint16_t fb_iqr[12];                        /* ADC spread (interquartile range) */
static float    fb_deg[12];                        /* calibrated angle                 */
static float    fb_min[12], fb_max[12];            /* range seen since the last 'r'    */
static bool     print_all_raw;
static char     last_stop[112] = "none since reset";

typedef struct {
    float    track_max;     /* max |error| while powered            */
    float    settle_sum;    /* sum of |error| at the end of holds   */
    float    settle_max;
    uint32_t settle_n;
} joint_stats_t;
static joint_stats_t stats[LEG_COUNT][JOINTS_PER_LEG];

/* A planned move of all joints from the current command to `to`, then a hold. */
typedef struct {
    float    from[LEG_COUNT][JOINTS_PER_LEG];
    float    to[LEG_COUNT][JOINTS_PER_LEG];
    uint32_t start_ms;
    uint32_t move_ms;
    uint32_t hold_ms;
} segment_t;

void Error_Handler(void);

/* ========================= Serial (interrupt driven) ======================= */

#define TX_BUF_SIZE 4096u
static char              tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head, tx_tail;   /* head: main loop only, tail: ISR only */
static volatile uint8_t  rx_char;

/* Queue text for transmission; never blocks (drops text if the buffer is full).
 * Call from the main loop only. */
static void uart_puts(const char *s)
{
    while (*s) {
        uint16_t next = (uint16_t)((tx_head + 1u) % TX_BUF_SIZE);
        if (next == tx_tail) break;
        tx_buf[tx_head] = *s++;
        tx_head = next;
    }
    USART3->CR1 |= USART_CR1_TXEIE;
}

void USART3_IRQHandler(void)
{
    uint32_t sr = USART3->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE))
        rx_char = (uint8_t)USART3->DR;               /* reading DR clears RXNE / ORE */
    if ((USART3->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
        if (tx_tail != tx_head) {
            USART3->DR = (uint8_t)tx_buf[tx_tail];
            tx_tail = (uint16_t)((tx_tail + 1u) % TX_BUF_SIZE);
        } else {
            USART3->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

static char uart_getc(void)
{
    char c = (char)rx_char;
    rx_char = 0;
    return c;
}

static void uart_int(int32_t v, bool show_plus)
{
    char buf[13];
    int i = sizeof buf - 1;
    uint32_t u = v < 0 ? 0u - (uint32_t)v : (uint32_t)v;
    buf[i] = '\0';
    do { buf[--i] = (char)('0' + u % 10u); u /= 10u; } while (u);
    if (v < 0)          buf[--i] = '-';
    else if (show_plus) buf[--i] = '+';
    uart_puts(&buf[i]);
}

static int32_t round_i(float v) { return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f); }

/* Prints v with one decimal, e.g. 94.0, -0.4 (with '+' for positive values if show_plus) */
static void uart_dec1(float v, bool show_plus)
{
    int32_t t = round_i(v * 10.0f);
    uint32_t u = t < 0 ? 0u - (uint32_t)t : (uint32_t)t;
    char frac[3] = { '.', (char)('0' + u % 10u), '\0' };
    if (t < 0)          uart_puts("-");
    else if (show_plus) uart_puts("+");
    uart_int((int32_t)(u / 10u), false);
    uart_puts(frac);
}

/* ============================== Utilities ============================== */

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float absf(float v) { return v < 0.0f ? -v : v; }
static bool  leg_selected(int l) { return (TEST_LEGS & LEG_BIT(l)) != 0; }
static uint8_t servo_of(int l, int j) { return LEGS[l].servo[j]; }

static void led(uint16_t pin, bool on) { HAL_GPIO_WritePin(GPIOB, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

/* Next selected leg after l, wrapping around (returns l itself if it is the only one). */
static int next_leg(int l)
{
    for (int i = 1; i <= LEG_COUNT; i++) {
        int n = (l + i) % LEG_COUNT;
        if (leg_selected(n)) return n;
    }
    return l;
}

static int first_leg(void) { return next_leg(LEG_COUNT - 1); }

/* The idx-th joint of the selected legs (leg by leg, body/femur/tibia). */
static bool joint_at(int idx, int *pl, int *pj)
{
    int k = 0;
    for (int l = 0; l < LEG_COUNT; l++) {
        if (!leg_selected(l)) continue;
        for (int j = 0; j < JOINTS_PER_LEG; j++, k++)
            if (k == idx) { *pl = l; *pj = j; return true; }
    }
    return false;
}

static void print_legs(uint8_t mask)
{
    bool first = true;
    for (int l = 0; l < LEG_COUNT; l++) {
        if (!(mask & LEG_BIT(l))) continue;
        uart_puts(first ? "" : "+");
        uart_puts(LEGS[l].tag);
        first = false;
    }
}

/* Rising edge on USER, debounced over 3 consecutive samples (30 ms). */
static bool button_pressed(void)
{
    static uint8_t high_count;
    static bool    was_down;
    if (HAL_GPIO_ReadPin(USER_BTN_PORT, USER_BTN_PIN) == GPIO_PIN_SET) {
        if (high_count < 3) high_count++;
    } else {
        high_count = 0;
    }
    bool down = high_count >= 3;
    bool event = down && !was_down;
    was_down = down;
    return event;
}

/* =============================== Feedback ============================== */

/* ADC code -> degrees, linear interpolation over the mean of the down/up tables. */
static float adc_to_deg(int n, float code)
{
    const float (*t)[MED_CALIB] = CALIB_TABLE[n];
    float prev_code = 0.0f, prev_deg = 0.0f;
    for (int k = 0; k < MED_CALIB; k++) {
        float c = (t[0][k] + t[1][k]) * 0.5f / MEAS_PER_ANG;
        if (code <= c) {
            if (k == 0) return ANGLES[0];
            return prev_deg + (ANGLES[k] - prev_deg) * (code - prev_code) / (c - prev_code);
        }
        prev_code = c;
        prev_deg  = ANGLES[k];
    }
    return MAX_TABLE_ANGLE;
}

/* Median of the window; *iqr = spread between the 25 % and 75 % samples. */
static uint16_t median_of_channel(int n, uint16_t *iqr)
{
    uint16_t v[ADC_SCANS];
    for (int i = 0; i < ADC_SCANS; i++) v[i] = adc_buf[i][n];
    for (int i = 1; i < ADC_SCANS; i++) {            /* insertion sort */
        uint16_t x = v[i];
        int k = i - 1;
        while (k >= 0 && v[k] > x) { v[k + 1] = v[k]; k--; }
        v[k + 1] = x;
    }
    *iqr = (uint16_t)(v[ADC_SCANS * 3 / 4] - v[ADC_SCANS / 4]);
    return (uint16_t)((v[ADC_SCANS / 2 - 1] + v[ADC_SCANS / 2] + 1u) / 2u);
}

static void range_reset(void)
{
    for (int n = 0; n < 12; n++) fb_min[n] = fb_max[n] = fb_deg[n];
}

static void feedback_update(void)
{
    for (int n = 0; n < 12; n++) {
        fb_raw[n] = median_of_channel(n, &fb_iqr[n]);
        fb_deg[n] = adc_to_deg(n, fb_raw[n]);
        if (fb_deg[n] < fb_min[n]) fb_min[n] = fb_deg[n];
        if (fb_deg[n] > fb_max[n]) fb_max[n] = fb_deg[n];
    }
}

static void set_last_stop(const char *what, const char *servo)
{
    strcpy(last_stop, what);
    if (servo) {
        strcat(last_stop, " (");
        strcat(last_stop, servo);
        strcat(last_stop, ")");
    }
}

/* A pot reading is trusted if it is steady and inside the calibration table
 * (a disconnected or shorted pot reads near 0 or full scale). */
static bool fb_valid(int n)
{
    const float (*t)[MED_CALIB] = CALIB_TABLE[n];
    float lo = (t[0][0] + t[1][0]) * 0.5f / MEAS_PER_ANG;
    float hi = (t[0][MED_CALIB - 1] + t[1][MED_CALIB - 1]) * 0.5f / MEAS_PER_ANG;
    return fb_iqr[n] <= FB_MAX_IQR && fb_raw[n] > lo && fb_raw[n] < hi;
}

/* Measured minus sent angle, corrected by the pot offset once it is known. */
static float joint_err(int l, int j)
{
    return fb_deg[servo_of(l, j)] - out_deg[l][j] - offset_deg[l][j];
}

/* ============================ Servo output ============================= */

static uint16_t deg_to_us(float deg)
{
    return (uint16_t)(clampf(US_AT_0_DEG + deg * US_PER_DEG, PULSE_MIN_US, PULSE_MAX_US) + 0.5f);
}

/* Neutral + offset (clamped to the joint limits) for joint j of leg l. */
static float pose_deg(int l, int j, float offset)
{
    const servo_t *s = &SERVOS[servo_of(l, j)];
    return s->neutral_deg + s->dir * clampf(offset, JOINT_LIMIT_DEG[j][0], JOINT_LIMIT_DEG[j][1]);
}

/* CCR preload is enabled, so a new value is latched at the next frame start:
 * a pulse in progress is never cut short. A compare value of 0 means no pulse. */
static void set_pulse(int l, int j, uint16_t us)
{
    uint8_t n = servo_of(l, j);
    __HAL_TIM_SET_COMPARE(&htim[n / 4], TIM_CH[n % 4], us);
}

/* Moves every sent command towards its planned command, at most
 * MAX_SLEW_DEG_S, and outputs it. This is the only place pulses are set. */
static void output_update(void)
{
    const float max_step = MAX_SLEW_DEG_S * (float)UPDATE_PERIOD_MS / 1000.0f;
    for (int l = 0; l < LEG_COUNT; l++) {
        if (!leg_selected(l)) continue;
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            if (!joint_on[l][j]) { set_pulse(l, j, 0); continue; }
            out_deg[l][j] += clampf(cmd_deg[l][j] - out_deg[l][j], -max_step, max_step);
            set_pulse(l, j, deg_to_us(out_deg[l][j]));
        }
    }
}

/* Switches a joint on at the angle its pot reads now: no jump. */
static void joint_enable(int l, int j)
{
    float now_deg = fb_deg[servo_of(l, j)];
    cmd_deg[l][j]      = now_deg;
    out_deg[l][j]      = now_deg;
    offset_deg[l][j]   = 0.0f;
    offset_known[l][j] = false;
    err_avg[l][j]      = 0.0f;
    joint_on[l][j]     = true;
    set_pulse(l, j, deg_to_us(now_deg));
}

static void outputs_off(void)
{
    for (int l = 0; l < LEG_COUNT; l++) {
        if (!leg_selected(l)) continue;
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            joint_on[l][j]     = false;
            offset_known[l][j] = false;
            set_pulse(l, j, 0);
        }
    }
}

/* ============================ Motion planning ========================== */

static float ease(float t)  /* smoothstep: zero speed at both ends, peak 1.5x mean */
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

/* Plans a move from the current command to `to`. Its time is set so that the
 * joint moving furthest peaks at speed_deg_s. */
static void segment_plan_at(segment_t *sg, const float to[LEG_COUNT][JOINTS_PER_LEG],
                            uint32_t hold_ms, uint32_t now, float speed_deg_s)
{
    float max_delta = 0.0f;
    memcpy(sg->from, cmd_deg, sizeof sg->from);
    memcpy(sg->to, to, sizeof sg->to);
    for (int l = 0; l < LEG_COUNT; l++) {
        if (!leg_selected(l)) continue;
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            float d = absf(sg->to[l][j] - sg->from[l][j]);
            if (d > max_delta) max_delta = d;
        }
    }
    sg->move_ms = 0;
    if (max_delta > 0.05f) {
        sg->move_ms = (uint32_t)(1.5f * max_delta / speed_deg_s * 1000.0f);
        if (sg->move_ms < MIN_MOVE_MS) sg->move_ms = MIN_MOVE_MS;
    }
    sg->hold_ms  = hold_ms;
    sg->start_ms = now;
}

static void segment_plan(segment_t *sg, const float to[LEG_COUNT][JOINTS_PER_LEG],
                         uint32_t hold_ms, uint32_t now)
{
    segment_plan_at(sg, to, hold_ms, now, MAX_SPEED_DEG_S);
}

/* Updates the planned command; returns true once the move and the hold are done. */
static bool segment_step(const segment_t *sg, uint32_t now)
{
    uint32_t elapsed = now - sg->start_ms;
    float k = sg->move_ms ? ease((float)elapsed / (float)sg->move_ms) : 1.0f;
    for (int l = 0; l < LEG_COUNT; l++)
        for (int j = 0; j < JOINTS_PER_LEG; j++)
            cmd_deg[l][j] = sg->from[l][j] + (sg->to[l][j] - sg->from[l][j]) * k;
    return elapsed >= sg->move_ms + sg->hold_ms;
}

static void neutral_targets(float to[LEG_COUNT][JOINTS_PER_LEG])
{
    for (int l = 0; l < LEG_COUNT; l++)
        for (int j = 0; j < JOINTS_PER_LEG; j++)
            to[l][j] = SERVOS[servo_of(l, j)].neutral_deg;
}

static uint8_t moving_legs(int current_leg)
{
    return TEST_MODE == MODE_ONE_BY_ONE ? LEG_BIT(current_leg) : TEST_LEGS;
}

/* Plans keyframe `step` for the legs in legs_mask; the other legs go to neutral. */
static void start_keyframe(segment_t *sg, uint32_t step, uint8_t legs_mask, uint32_t now)
{
    const keyframe_t *kf = &SEQUENCE[step];
    float to[LEG_COUNT][JOINTS_PER_LEG];
    for (int l = 0; l < LEG_COUNT; l++)
        for (int j = 0; j < JOINTS_PER_LEG; j++)
            to[l][j] = pose_deg(l, j, (legs_mask & LEG_BIT(l)) ? kf->deg[j] : 0.0f);
    segment_plan(sg, to, kf->hold_ms, now);

    /* step 5/19  FL  body +0  femur +20  tibia +0 deg  move 3000 ms  hold 500 ms  -> FL 1171/979/1164 us */
    uart_puts("step ");
    uart_int((int32_t)step + 1, false);
    uart_puts("/");
    uart_int((int32_t)SEQUENCE_LEN, false);
    uart_puts("  ");
    print_legs(legs_mask);
    for (int j = 0; j < JOINTS_PER_LEG; j++) {
        uart_puts("  ");
        uart_puts(JOINT_NAME[j]);
        uart_puts(" ");
        uart_int(round_i(clampf(kf->deg[j], JOINT_LIMIT_DEG[j][0], JOINT_LIMIT_DEG[j][1])), true);
    }
    uart_puts(" deg  move ");
    uart_int((int32_t)sg->move_ms, false);
    uart_puts(" ms  hold ");
    uart_int((int32_t)sg->hold_ms, false);
    uart_puts(" ms  ->");
    for (int l = 0; l < LEG_COUNT; l++) {
        if (!(legs_mask & LEG_BIT(l))) continue;
        uart_puts(" ");
        uart_puts(LEGS[l].tag);
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uart_puts(j ? "/" : " ");
            uart_int(deg_to_us(sg->to[l][j]), false);
        }
    }
    uart_puts(" us\r\n");
}

/* ======================== Monitoring and logging ======================= */

/* <sent>/<meas>/<err> in degrees; "off" when the servo gets no pulses. */
static void print_joint_fb(int l, int j)
{
    uint8_t n = servo_of(l, j);
    if (joint_on[l][j]) {
        uart_dec1(out_deg[l][j], false);
        uart_puts("/");
        uart_dec1(fb_deg[n], false);
        uart_puts("/");
        uart_dec1(joint_err(l, j), false);
    } else {
        uart_puts("off/");
        uart_dec1(fb_deg[n], false);
        uart_puts("/-");
    }
}

static void print_telemetry(uint32_t now)
{
    uart_puts("fb ");
    uart_dec1((float)now / 1000.0f, false);
    for (int l = 0; l < LEG_COUNT; l++) {
        if (!leg_selected(l)) continue;
        uart_puts(" ");
        uart_puts(LEGS[l].tag);
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uart_puts(" ");
            print_joint_fb(l, j);
        }
        uart_puts(" adc");
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uart_puts(j ? "/" : " ");
            uart_int(fb_raw[servo_of(l, j)], false);
        }
    }
    if (print_all_raw) {
        uart_puts(" all");
        for (int n = 0; n < 12; n++) {
            uart_puts(" ");
            uart_int(fb_raw[n], false);
        }
    }
    uart_puts("\r\n");
}

/* Updates the averaged error of every powered joint with a known offset and
 * returns true (and reports it) if one is above FAULT_ERR_DEG. */
static bool fault_detected(void)
{
    const float alpha = (float)UPDATE_PERIOD_MS / FAULT_TAU_MS;
    for (int l = 0; l < LEG_COUNT; l++) {
        if (!leg_selected(l)) continue;
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            if (!joint_on[l][j] || !offset_known[l][j]) continue;
            float e = absf(joint_err(l, j));
            err_avg[l][j] += (e - err_avg[l][j]) * alpha;
            if (e > stats[l][j].track_max) stats[l][j].track_max = e;
            if (err_avg[l][j] > FAULT_ERR_DEG) {
                uart_puts("FAULT: ");
                uart_puts(SERVOS[servo_of(l, j)].name);
                uart_puts(" not following (sent ");
                uart_dec1(out_deg[l][j], false);
                uart_puts(", meas ");
                uart_dec1(fb_deg[servo_of(l, j)], false);
                uart_puts(", avg error ");
                uart_dec1(err_avg[l][j], false);
                uart_puts(" deg): blocked, stalled or oscillating -> all pulses off\r\n");
                set_last_stop("fault cut-out: joint not following its command", SERVOS[servo_of(l, j)].name);
                return true;
            }
        }
    }
    return false;
}

/* End of a hold: log the settled error of the legs in legs_mask and add it to the stats. */
static void log_settled(uint8_t legs_mask, const char *label)
{
    uart_puts("settled ");
    uart_puts(label);
    for (int l = 0; l < LEG_COUNT; l++) {
        if (!(legs_mask & LEG_BIT(l))) continue;
        uart_puts(" ");
        uart_puts(LEGS[l].tag);
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uart_puts(" ");
            uart_puts(SERVOS[servo_of(l, j)].name);
            uart_puts(" ");
            print_joint_fb(l, j);
            if (joint_on[l][j]) {
                float e = absf(joint_err(l, j));
                stats[l][j].settle_sum += e;
                stats[l][j].settle_n++;
                if (e > stats[l][j].settle_max) stats[l][j].settle_max = e;
            }
        }
    }
    uart_puts("\r\n");
}

static void print_and_reset_stats(const char *title)
{
    uart_puts("stats (");
    uart_puts(title);
    uart_puts("), |error| in deg after the pot offset:\r\n");
    for (int l = 0; l < LEG_COUNT; l++) {
        if (!leg_selected(l)) continue;
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            const joint_stats_t *s = &stats[l][j];
            uart_puts("  ");
            uart_puts(SERVOS[servo_of(l, j)].name);
            uart_puts("  pot offset ");
            uart_dec1(offset_deg[l][j], true);
            uart_puts("  settled mean ");
            uart_dec1(s->settle_n ? s->settle_sum / (float)s->settle_n : 0.0f, false);
            uart_puts(" max ");
            uart_dec1(s->settle_max, false);
            uart_puts(" (");
            uart_int((int32_t)s->settle_n, false);
            uart_puts(" holds)  tracking max ");
            uart_dec1(s->track_max, false);
            uart_puts("\r\n");
        }
    }
    memset(stats, 0, sizeof stats);
}

/* =================== Troubleshooting program (PROGRAM_CHECK) =================== */
/* One servo at a time: pot check with pulses off, then the first pulse at the
 * servo's own measured angle while every pot of the leg is watched at 1 ms (cut
 * at once if anything moves), then small slow sweeps around that position to
 * measure the pot offset and gain. The servo goes limp again before the next one.
 * Neutral angles and joint limits are not used, so a wrong neutral cannot hurt. */

#define CK_AMPS (sizeof CHECK_AMPLITUDE_DEG / sizeof CHECK_AMPLITUDE_DEG[0])

typedef enum { CK_NOT_RUN, CK_PASS, CK_GAIN, CK_POT_INVALID, CK_PROBE_MOVED, CK_FAULT, CK_STOPPED } ck_status_t;
static const char *const CK_STATUS_TEXT[] = {
    "not run", "PASS", "GAIN MISMATCH", "POT INVALID", "MOVED ON FIRST PULSE", "FAULT (not following)", "stopped",
};

typedef struct {
    ck_status_t status;
    float start_deg, offset, track_max, other_max;
    float gain[CK_AMPS];
    float probe_move[JOINTS_PER_LEG];   /* pot changes when the first pulse was cut */
    float probe_peak[JOINTS_PER_LEG];   /* largest pot change seen during the probe */
} ck_result_t;

static uint32_t ck_next_tick, ck_next_telemetry;

/* Median of the last 8 complete ADC scans of servo n (about 2 ms), in degrees.
 * Used while probing, where the 16 ms window would react too late. */
static float fast_fb_deg(int n)
{
    uint32_t written = ADC_SCANS * 12u - __HAL_DMA_GET_COUNTER(&hdma_adc1);
    int s = (int)(written / 12u) - 1;          /* last complete scan */
    uint16_t v[8];
    for (int i = 0; i < 8; i++, s--) {
        if (s < 0) s += ADC_SCANS;
        v[i] = adc_buf[s][n];
    }
    for (int i = 1; i < 8; i++) {
        uint16_t x = v[i];
        int k = i - 1;
        while (k >= 0 && v[k] > x) { v[k + 1] = v[k]; k--; }
        v[k + 1] = x;
    }
    return adc_to_deg(n, (v[3] + v[4]) * 0.5f);
}

/* One 10 ms step: feedback, outputs, telemetry. False if USER or 'x' asked to stop. */
static bool check_tick(void)
{
    while ((int32_t)(HAL_GetTick() - ck_next_tick) < 0)
        __WFI();
    ck_next_tick += UPDATE_PERIOD_MS;
    uint32_t now = HAL_GetTick();
    char c = uart_getc();
    bool pressed = button_pressed();
    feedback_update();
    output_update();
    if ((int32_t)(now - ck_next_telemetry) >= 0) {
        ck_next_telemetry += TELEMETRY_MS;
        print_telemetry(now);
    }
    return !(pressed || c == 'x' || c == 'X');
}

static bool check_wait(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms)
        if (!check_tick()) return false;
    return true;
}

/* First pulse at the measured angle, watching all three pots of the leg at 1 ms.
 * Returns false (pulses already cut) if any pot stayed more than CHECK_PROBE_CUT_DEG
 * off for CHECK_PROBE_SUSTAIN_MS, or went more than CHECK_PROBE_FAST_DEG off. */
static bool check_probe(int l, int j, ck_result_t *r)
{
    float    ref[JOINTS_PER_LEG];
    uint32_t over_ms[JOINTS_PER_LEG] = { 0 };
    for (int k = 0; k < JOINTS_PER_LEG; k++) {
        ref[k] = fast_fb_deg(servo_of(l, k));
        r->probe_peak[k] = 0.0f;
    }
    joint_enable(l, j);
    uint32_t t0 = HAL_GetTick(), t = t0;
    while (t - t0 < CHECK_PROBE_MS) {
        bool cut = false;
        for (int k = 0; k < JOINTS_PER_LEG; k++) {
            float d = fast_fb_deg(servo_of(l, k)) - ref[k];
            r->probe_move[k] = d;
            if (absf(d) > absf(r->probe_peak[k])) r->probe_peak[k] = d;
            over_ms[k] = absf(d) > CHECK_PROBE_CUT_DEG ? over_ms[k] + 1u : 0u;
            if (over_ms[k] >= CHECK_PROBE_SUSTAIN_MS || absf(d) > CHECK_PROBE_FAST_DEG) cut = true;
        }
        if (cut) {
            set_pulse(l, j, 0);          /* takes effect at the end of this frame */
            joint_on[l][j] = false;
            return false;
        }
        while (HAL_GetTick() == t) { }
        t = HAL_GetTick();
    }
    return true;
}

static ck_result_t check_one(int l, int j)
{
    ck_result_t r;
    memset(&r, 0, sizeof r);
    uint8_t n = servo_of(l, j);
    const servo_t *s = &SERVOS[n];
    float other_ref[JOINTS_PER_LEG];
    float to[LEG_COUNT][JOINTS_PER_LEG];
    segment_t sg;

    uart_puts("--- ");
    uart_puts(s->name);
    uart_puts(" (");
    uart_puts(JOINT_NAME[j]);
    uart_puts(", servo ");
    uart_int(n, false);
    uart_puts(", PWM ");
    uart_puts(s->pin_name);
    uart_puts(", pot ");
    uart_puts(s->fb_pin);
    uart_puts(") ---\r\n");
    if (!check_wait(300)) { r.status = CK_STOPPED; return r; }

    uart_puts("  pulses off: pot reads ");
    uart_dec1(fb_deg[n], false);
    uart_puts(" deg (adc ");
    uart_int(fb_raw[n], false);
    uart_puts(", spread ");
    uart_int(fb_iqr[n], false);
    uart_puts(")\r\n");
    if (!fb_valid(n)) {
        uart_puts("  pot reading invalid (out of the calibration range or noisy): servo not switched on\r\n");
        r.status = CK_POT_INVALID;
        return r;
    }
    r.start_deg = fb_deg[n];

    if (!check_probe(l, j, &r)) {
        uart_puts("  first pulse at ");
        uart_dec1(r.start_deg, false);
        uart_puts(" deg: pots moved");
        for (int k = 0; k < JOINTS_PER_LEG; k++) {
            uart_puts(" ");
            uart_puts(SERVOS[servo_of(l, k)].name);
            uart_puts(" ");
            uart_dec1(r.probe_move[k], true);
        }
        uart_puts(" deg -> pulses cut\r\n");
        r.status = CK_PROBE_MOVED;
        check_wait(500);
        return r;
    }
    feedback_update();                  /* the probe did not update the 16 ms medians */
    r.offset = fb_deg[n] - out_deg[l][j];
    offset_deg[l][j]   = r.offset;
    offset_known[l][j] = true;
    uart_puts("  holds its first pulse at ");
    uart_dec1(out_deg[l][j], false);
    uart_puts(" deg, pot offset ");
    uart_dec1(r.offset, true);
    uart_puts(" deg\r\n");
    for (int k = 0; k < JOINTS_PER_LEG; k++) other_ref[k] = fb_deg[servo_of(l, k)];

    for (unsigned a = 0; a < CK_AMPS; a++) {
        const float A = CHECK_AMPLITUDE_DEG[a];
        const float target[3] = { r.start_deg + A, r.start_deg - A, r.start_deg };
        float settled[3];
        for (int t = 0; t < 3; t++) {
            memcpy(to, cmd_deg, sizeof to);
            to[l][j] = target[t];
            segment_plan_at(&sg, to, CHECK_HOLD_MS, HAL_GetTick(), CHECK_SPEED_DEG_S);
            for (;;) {
                bool done = segment_step(&sg, HAL_GetTick());
                if (!check_tick()) { r.status = CK_STOPPED; goto off; }
                if (fault_detected()) { r.status = CK_FAULT; goto off; }
                float e = absf(joint_err(l, j));
                if (e > r.track_max) r.track_max = e;
                for (int k = 0; k < JOINTS_PER_LEG; k++) {
                    if (k == j) continue;
                    float d = absf(fb_deg[servo_of(l, k)] - other_ref[k]);
                    if (d > r.other_max) r.other_max = d;
                }
                if (done) break;
            }
            settled[t] = fb_deg[n];
            uart_puts("  sent ");
            uart_dec1(out_deg[l][j], false);
            uart_puts(" (");
            uart_dec1(target[t] - r.start_deg, true);
            uart_puts(")  pot ");
            uart_dec1(settled[t], false);
            uart_puts("  error ");
            uart_dec1(joint_err(l, j), true);
            uart_puts("\r\n");
        }
        r.gain[a] = (settled[0] - settled[1]) / (2.0f * A);
        uart_puts("  +/-");
        uart_int(round_i(A), false);
        uart_puts(" deg sweep: pot moved ");
        uart_dec1(settled[0] - settled[1], false);
        uart_puts(" deg for ");
        uart_int(round_i(2.0f * A), false);
        uart_puts(" commanded -> gain ");
        uart_dec1(r.gain[a], false);
        uart_puts(r.gain[a] >= CHECK_GAIN_MIN && r.gain[a] <= CHECK_GAIN_MAX ? " ok\r\n"
                  : (a + 1 < CK_AMPS ? " low/high (small sweep: backlash?)\r\n" : " OUT OF RANGE\r\n"));
    }
    /* Judge the gain on the largest sweep: small sweeps are dominated by backlash/deadband. */
    r.status = (r.gain[CK_AMPS - 1] >= CHECK_GAIN_MIN && r.gain[CK_AMPS - 1] <= CHECK_GAIN_MAX) ? CK_PASS : CK_GAIN;
off:
    outputs_off();
    uart_puts("  pulses off, result: ");
    uart_puts(CK_STATUS_TEXT[r.status]);
    uart_puts("\r\n");
    return r;
}

static void run_check(void)
{
    static ck_result_t res[LEG_COUNT][JOINTS_PER_LEG];
    bool stopped = false;

    memset(res, 0, sizeof res);
    ck_next_tick = ck_next_telemetry = HAL_GetTick();
    led(LED_BLUE, false);
    led(LED_RED, false);
    led(LED_GREEN, true);
    uart_puts("=== servo check: one servo at a time, small slow moves around where it is now ===\r\n");

    for (int l = 0; l < LEG_COUNT && !stopped; l++) {
        if (!leg_selected(l)) continue;
        for (int j = 0; j < JOINTS_PER_LEG && !stopped; j++) {
            if (!(CHECK_JOINTS & (1u << j))) continue;
            res[l][j] = check_one(l, j);
            if (res[l][j].status == CK_STOPPED || !check_wait(500)) stopped = true;
        }
    }
    outputs_off();

    uart_puts("=== servo check summary ===\r\n");
    bool all_pass = !stopped;
    for (int l = 0; l < LEG_COUNT; l++) {
        if (!leg_selected(l)) continue;
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            const ck_result_t *r = &res[l][j];
            if (!(CHECK_JOINTS & (1u << j))) continue;
            uart_puts("  ");
            uart_puts(SERVOS[servo_of(l, j)].name);
            uart_puts("  ");
            uart_puts(CK_STATUS_TEXT[r->status]);
            if (r->status == CK_PASS || r->status == CK_GAIN || r->status == CK_FAULT) {
                uart_puts("  start ");
                uart_dec1(r->start_deg, false);
                uart_puts("  pot offset ");
                uart_dec1(r->offset, true);
                uart_puts("  gain");
                for (unsigned a = 0; a < CK_AMPS; a++) {
                    uart_puts(a ? "/" : " ");
                    uart_dec1(r->gain[a], false);
                }
                uart_puts("  tracking max ");
                uart_dec1(r->track_max, false);
                uart_puts("  other pots moved max ");
                uart_dec1(r->other_max, false);
            } else if (r->status == CK_PROBE_MOVED) {
                uart_puts("  pots moved");
                for (int k = 0; k < JOINTS_PER_LEG; k++) {
                    uart_puts(" ");
                    uart_dec1(r->probe_move[k], true);
                }
                uart_puts(" deg on the first pulse");
            }
            uart_puts("\r\n");
            if (r->status != CK_PASS) all_pass = false;
        }
    }
    set_last_stop(all_pass ? "servo check finished: all PASS" : "servo check finished: see summary", NULL);
    led(LED_GREEN, false);
    led(LED_RED, !all_pass);
    uart_puts(all_pass ? "all servos PASS\r\n" : "not all servos passed, see above\r\n");
}

/* =================== Mechanical limit search (PROGRAM_LIMITS) =================== */
/* All joints of the leg are switched on one at a time with the safe probe and then
 * hold where they are. Each joint in turn moves at LIMIT_SPEED_DEG_S until its pot
 * lags LIMIT_LAG_DEG behind the command for LIMIT_DETECT_MS: that is its limit (blocked,
 * or it cannot follow any more). It stops pushing at once, backs off, returns to the
 * start and repeats in the other direction. Angles are on the PWM/pot scale. */

typedef struct {
    bool     done;
    float    start;                 /* command at the start (deg)                     */
    float    lim[2];                /* [0] low, [1] high: pot angle at the limit       */
    float    lim_cmd[2];            /* the same limits as commands (pot - offset)      */
    bool     blocked[2];            /* true: blocked; false: travel cap reached        */
} lim_result_t;

enum { RUN_OK = 0, RUN_STOPPED = -1, RUN_FAULT = -2 };

/* Runs a planned segment to its end. */
static int run_segment(segment_t *sg)
{
    for (;;) {
        bool done = segment_step(sg, HAL_GetTick());
        if (!check_tick()) return RUN_STOPPED;
        if (fault_detected()) return RUN_FAULT;
        if (done) return RUN_OK;
    }
}

/* Plans and runs a move of joint (l, j) to `target` at speed_deg_s; the others hold. */
static int move_joint(int l, int j, float target, float speed_deg_s)
{
    float to[LEG_COUNT][JOINTS_PER_LEG];
    segment_t sg;
    memcpy(to, cmd_deg, sizeof to);
    to[l][j] = target;
    segment_plan_at(&sg, to, 200, HAL_GetTick(), speed_deg_s);
    return run_segment(&sg);
}

/* Moves joint (l, j) at constant speed in direction dir (+1/-1) until it is blocked
 * (returns 1) or the travel cap is reached (returns 0). *pot / *cmd = where it ended. */
static int limit_search(int l, int j, int dir, float start, float *pot, float *cmd)
{
    const float step = LIMIT_SPEED_DEG_S * (float)UPDATE_PERIOD_MS / 1000.0f;
    uint8_t  n = servo_of(l, j);
    uint32_t lag_ms = 0;

    /* This joint is expected to lag here, so take it out of the fault cut-out while
     * searching (the other joints stay protected). */
    offset_known[l][j] = false;
    for (;;) {
        float next = cmd_deg[l][j] + (float)dir * step;
        bool  cap  = absf(next - start) > LIMIT_MAX_TRAVEL_DEG || next < LIMIT_MIN_DEG || next > LIMIT_MAX_DEG;
        if (!cap) cmd_deg[l][j] = next;
        if (!check_tick()) { offset_known[l][j] = true; return RUN_STOPPED; }
        if (fault_detected()) { offset_known[l][j] = true; return RUN_FAULT; }

        float lag = (float)dir * (out_deg[l][j] + offset_deg[l][j] - fb_deg[n]);
        lag_ms = lag > LIMIT_LAG_DEG ? lag_ms + UPDATE_PERIOD_MS : 0;
        if (lag_ms >= LIMIT_DETECT_MS || (cap && lag_ms == 0)) {
            *pot = fb_deg[n];
            *cmd = out_deg[l][j];
            /* Stop pushing at once: command the joint where it actually is. */
            cmd_deg[l][j] = out_deg[l][j] = fb_deg[n] - offset_deg[l][j];
            offset_known[l][j] = true;
            err_avg[l][j] = 0.0f;
            return lag_ms >= LIMIT_DETECT_MS ? 1 : 0;
        }
    }
}

static void print_limit_line(int l, int j, const lim_result_t *r)
{
    const servo_t *s = &SERVOS[servo_of(l, j)];
    uart_puts("  ");
    uart_puts(s->name);
    if (!r->done) { uart_puts("  not measured\r\n"); return; }
    uart_puts("  start ");
    uart_dec1(r->start, false);
    uart_puts("  limits ");
    uart_dec1(r->lim_cmd[0], false);
    uart_puts(" .. ");
    uart_dec1(r->lim_cmd[1], false);
    uart_puts(" deg (");
    uart_int(deg_to_us(r->lim_cmd[0]), false);
    uart_puts(" .. ");
    uart_int(deg_to_us(r->lim_cmd[1]), false);
    uart_puts(" us)  range ");
    uart_dec1(r->lim_cmd[1] - r->lim_cmd[0], false);
    uart_puts(r->blocked[0] ? "  low: blocked" : "  low: travel cap");
    uart_puts(r->blocked[1] ? ", high: blocked" : ", high: travel cap");
    uart_puts("  neutral ");
    uart_int(round_i(s->neutral_deg), false);
    uart_puts(s->neutral_deg >= r->lim_cmd[0] && s->neutral_deg <= r->lim_cmd[1] ? " inside" : " OUTSIDE");
    uart_puts("\r\n");
}

static void run_limits(void)
{
    static lim_result_t res[LEG_COUNT][JOINTS_PER_LEG];
    ck_result_t probe;
    int rc = RUN_OK;

    memset(res, 0, sizeof res);
    ck_next_tick = ck_next_telemetry = HAL_GetTick();
    led(LED_BLUE, false);
    led(LED_RED, false);
    led(LED_GREEN, true);
    uart_puts("=== limit search: joints held where they are, one joint at a time moves slowly to each limit ===\r\n");

    /* 1. All pots of the selected legs must be valid. */
    if (!check_wait(300)) rc = RUN_STOPPED;
    for (int l = 0; l < LEG_COUNT && rc == RUN_OK; l++) {
        if (!leg_selected(l)) continue;
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uint8_t n = servo_of(l, j);
            if (fb_valid(n)) continue;
            uart_puts("  ");
            uart_puts(SERVOS[n].name);
            uart_puts(" pot invalid (adc ");
            uart_int(fb_raw[n], false);
            uart_puts("): not starting\r\n");
            set_last_stop("limit search refused: invalid pot reading", SERVOS[n].name);
            rc = RUN_FAULT;
        }
    }

    /* 2. Switch every joint on at its measured angle with the safe probe; they then hold. */
    for (int l = 0; l < LEG_COUNT && rc == RUN_OK; l++) {
        if (!leg_selected(l)) continue;
        for (int j = 0; j < JOINTS_PER_LEG && rc == RUN_OK; j++) {
            uint8_t n = servo_of(l, j);
            memset(&probe, 0, sizeof probe);
            if (!check_probe(l, j, &probe)) {
                uart_puts("  ");
                uart_puts(SERVOS[n].name);
                uart_puts(" moved on its first pulse (pot changes");
                for (int k = 0; k < JOINTS_PER_LEG; k++) {
                    uart_puts(" ");
                    uart_puts(SERVOS[servo_of(l, k)].name);
                    uart_puts(" ");
                    uart_dec1(probe.probe_move[k], true);
                }
                uart_puts(" deg): pulses cut, not starting\r\n");
                set_last_stop("limit search: servo moved on its first pulse", SERVOS[n].name);
                rc = RUN_FAULT;
                break;
            }
            feedback_update();
            offset_deg[l][j]   = fb_deg[n] - out_deg[l][j];
            offset_known[l][j] = true;
            uart_puts("  ");
            uart_puts(SERVOS[n].name);
            uart_puts(" on and holding at ");
            uart_dec1(out_deg[l][j], false);
            uart_puts(" deg, pot offset ");
            uart_dec1(offset_deg[l][j], true);
            uart_puts(" (peak pot change on first pulse:");
            for (int k = 0; k < JOINTS_PER_LEG; k++) {
                uart_puts(" ");
                uart_puts(SERVOS[servo_of(l, k)].name);
                uart_puts(" ");
                uart_dec1(probe.probe_peak[k], true);
            }
            uart_puts(")\r\n");
            if (!check_wait(300)) rc = RUN_STOPPED;
        }
    }

    /* 3. Search the limits of each joint, + then -, returning to the start in between. */
    for (int l = 0; l < LEG_COUNT && rc == RUN_OK; l++) {
        if (!leg_selected(l)) continue;
        for (int j = 0; j < JOINTS_PER_LEG && rc == RUN_OK; j++) {
            if (!(LIMIT_JOINTS & (1u << j))) continue;
            lim_result_t *r = &res[l][j];
            r->start = out_deg[l][j];
            uart_puts("--- ");
            uart_puts(SERVOS[servo_of(l, j)].name);
            uart_puts(": searching limits from ");
            uart_dec1(r->start, false);
            uart_puts(" deg ---\r\n");
            for (int side = 1; side >= 0 && rc == RUN_OK; side--) {
                int   dir = side ? +1 : -1;
                float pot = 0.0f, cmd = 0.0f;
                int   found = limit_search(l, j, dir, r->start, &pot, &cmd);
                if (found < 0) { rc = found; break; }
                r->lim[side]     = pot;
                r->lim_cmd[side] = pot - offset_deg[l][j];
                r->blocked[side] = found == 1;
                uart_puts(dir > 0 ? "  high: " : "  low:  ");
                uart_puts(found ? "blocked at pot " : "travel cap at pot ");
                uart_dec1(pot, false);
                uart_puts(" deg (command was ");
                uart_dec1(cmd, false);
                uart_puts("), backing off\r\n");
                rc = move_joint(l, j, out_deg[l][j] - (float)dir * LIMIT_BACKOFF_DEG, MAX_SPEED_DEG_S);
                if (rc == RUN_OK) rc = move_joint(l, j, r->start, MAX_SPEED_DEG_S);
            }
            if (rc == RUN_OK) r->done = true;
        }
    }

    outputs_off();
    uart_puts(rc == RUN_OK      ? "=== limit search done, pulses off ===\r\n"
              : rc == RUN_STOPPED ? "=== limit search stopped by user, pulses off ===\r\n"
                                  : "=== limit search aborted, pulses off ===\r\n");
    for (int l = 0; l < LEG_COUNT; l++) {
        if (!leg_selected(l)) continue;
        for (int j = 0; j < JOINTS_PER_LEG; j++)
            if (LIMIT_JOINTS & (1u << j)) print_limit_line(l, j, &res[l][j]);
    }
    if (rc == RUN_OK)      set_last_stop("limit search done", NULL);
    if (rc == RUN_STOPPED) set_last_stop("limit search stopped by user", NULL);
    led(LED_GREEN, false);
    led(LED_RED, rc != RUN_OK);
}

/* ============================ Peripherals ============================== */

static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* 8 MHz MCO from the ST-LINK (HSE bypass) -> PLL -> 96 MHz, as in quadruped_control_mcu. */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_BYPASS;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 4;
    osc.PLL.PLLN       = 96;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;
    osc.PLL.PLLQ       = 4;
    osc.PLL.PLLR       = 2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        /* No MCO clock: fall back to the internal 16 MHz HSI, same 96 MHz result. */
        osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
        osc.HSIState            = RCC_HSI_ON;
        osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
        osc.PLL.PLLM            = 8;
        if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
        clock_from_hsi = true;
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;    /* 48 MHz, timers x2 = 96 MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV1;    /* 96 MHz, ADC clock = /4 = 24 MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

static void gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, LED_GREEN | LED_BLUE | LED_RED, GPIO_PIN_RESET);
    g.Pin   = LED_GREEN | LED_BLUE | LED_RED;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    g.Pin  = USER_BTN_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;           /* external pull-down on the Nucleo */
    HAL_GPIO_Init(USER_BTN_PORT, &g);
}

static void uart_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_USART3_CLK_ENABLE();
    g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;   /* PD8 TX, PD9 RX -> ST-LINK VCP */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &g);

    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();

    USART3->CR1 |= USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* ADC1 scans the 12 pot inputs continuously; DMA2 Stream4 (as in quadruped_control_mcu)
 * fills adc_buf as a ring. No interrupts are used. */
static void adc_init(void)
{
    GPIO_InitTypeDef       g  = {0};
    ADC_ChannelConfTypeDef ch = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    g.Pin  = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin  = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
    HAL_GPIO_Init(GPIOC, &g);

    hdma_adc1.Instance                 = DMA2_Stream4;
    hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ENABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 12;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    ch.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    for (int n = 0; n < 12; n++) {
        ch.Channel = SERVOS[n].adc_ch;
        ch.Rank    = (uint32_t)n + 1u;
        if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) Error_Handler();
    }
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_SCANS * 12) != HAL_OK) Error_Handler();
    HAL_Delay(20);   /* fill the whole window once before the first median */
}

/* All three timers run, but only the selected legs' pins are connected to them.
 * Every other servo PWM pin stays an unconfigured input (no signal). */
static void pwm_init(void)
{
    static TIM_TypeDef *const INSTANCE[3] = { TIM2, TIM3, TIM4 };
    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq() * 2u;   /* APB1 prescaler is 2 */
    TIM_OC_InitTypeDef oc = {0};
    GPIO_InitTypeDef   g  = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    for (int t = 0; t < 3; t++) {
        htim[t].Instance               = INSTANCE[t];
        htim[t].Init.Prescaler         = tim_clk / 1000000u - 1u;   /* 1 us tick */
        htim[t].Init.CounterMode       = TIM_COUNTERMODE_UP;
        htim[t].Init.Period            = PWM_PERIOD_US;
        htim[t].Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
        htim[t].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        if (HAL_TIM_PWM_Init(&htim[t]) != HAL_OK) Error_Handler();
    }

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;                       /* no pulses until started */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    for (int l = 0; l < LEG_COUNT; l++) {
        if (!leg_selected(l)) continue;
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uint8_t n = servo_of(l, j);
            const servo_t *s = &SERVOS[n];

            if (HAL_TIM_PWM_ConfigChannel(&htim[n / 4], &oc, TIM_CH[n % 4]) != HAL_OK) Error_Handler();
            if (HAL_TIM_PWM_Start(&htim[n / 4], TIM_CH[n % 4]) != HAL_OK) Error_Handler();

            if (!MOTION_ENABLED) continue;   /* pin stays a driven-low output */
            g.Pin       = s->pin;
            g.Mode      = GPIO_MODE_AF_PP;
            g.Pull      = GPIO_NOPULL;
            g.Speed     = GPIO_SPEED_FREQ_LOW;
            g.Alternate = s->af;
            HAL_GPIO_Init(s->port, &g);
        }
    }
}

/* Drives all 12 servo signal lines low as early as possible after reset, so no
 * servo sees a floating (noisy) signal line. Pins of the selected legs are
 * handed to the timers later by pwm_init(). */
static void servo_pins_low(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    for (int n = 0; n < 12; n++) {
        HAL_GPIO_WritePin(SERVOS[n].port, SERVOS[n].pin, GPIO_PIN_RESET);
        g.Pin = SERVOS[n].pin;
        HAL_GPIO_Init(SERVOS[n].port, &g);
    }
}

/* 'r': the range each pot of the selected legs has read since the last 'r'. */
static void print_ranges(void)
{
    uart_puts("pot range since last 'r' (deg, min..max, now):");
    for (int l = 0; l < LEG_COUNT; l++) {
        if (!leg_selected(l)) continue;
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uint8_t n = servo_of(l, j);
            uart_puts("  ");
            uart_puts(SERVOS[n].name);
            uart_puts(" ");
            uart_dec1(fb_min[n], false);
            uart_puts("..");
            uart_dec1(fb_max[n], false);
            uart_puts(" now ");
            uart_dec1(fb_deg[n], false);
        }
    }
    uart_puts("\r\n");
    range_reset();
}

static void print_help(void)
{
    uart_puts("commands: s = start, x = stop, r = pot ranges since last r, a = toggle raw ADC of all 12 channels, h = help\r\n");
}

static void print_banner(void)
{
    uart_puts("\r\n=== MOSAC servo test (slow) + feedback: ");
    print_legs(TEST_LEGS);
    uart_puts(TEST_MODE == MODE_ONE_BY_ONE ? ", one leg at a time ===\r\n" : ", all together ===\r\n");
    if (!MOTION_ENABLED)
        uart_puts("*** MOTION DISABLED (MOTION_ENABLED 0): feedback only, no servo pulses ***\r\n");
    else if (PROGRAM == PROGRAM_CHECK)
        uart_puts("program: SERVO CHECK (one servo at a time, +/-5 and +/-10 deg around where it is, 5 deg/s)\r\n");
    else if (PROGRAM == PROGRAM_LIMITS)
        uart_puts("program: LIMIT SEARCH (joints hold; each moves at 5 deg/s until blocked, then backs off)\r\n");
    else
        uart_puts("program: test sequence around neutral\r\n");
    for (int l = 0; l < LEG_COUNT; l++) {
        if (!leg_selected(l)) continue;
        uart_puts(LEGS[l].name);
        uart_puts(":\r\n");
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uint8_t n = servo_of(l, j);
            const servo_t *s = &SERVOS[n];
            uart_puts("  ");
            uart_puts(JOINT_NAME[j]);
            uart_puts("  servo ");
            uart_int(n, false);
            uart_puts(n < 10 ? "  " : " ");
            uart_puts(s->name);
            uart_puts("  PWM TIM");
            uart_int(n / 4 + 2, false);
            uart_puts(" CH");
            uart_int(n % 4 + 1, false);
            uart_puts(" ");
            uart_puts(s->pin_name);
            uart_puts("  feedback ADC1 IN");
            uart_int((int32_t)s->adc_ch, false);
            uart_puts(" ");
            uart_puts(s->fb_pin);
            uart_puts("  neutral ");
            uart_int(round_i(s->neutral_deg), false);
            uart_puts(" deg (");
            uart_int(deg_to_us(s->neutral_deg), false);
            uart_puts(" us)  dir ");
            uart_puts(s->dir > 0 ? "+" : "-");
            uart_puts("\r\n");
        }
    }
    uart_puts("limits (deg from neutral): body ");
    for (int j = 0; j < JOINTS_PER_LEG; j++) {
        if (j) { uart_puts(", "); uart_puts(JOINT_NAME[j]); uart_puts(" "); }
        uart_int(round_i(JOINT_LIMIT_DEG[j][0]), true);
        uart_puts("/");
        uart_int(round_i(JOINT_LIMIT_DEG[j][1]), true);
    }
    uart_puts("\r\nspeed: moves peak at ");
    uart_int(round_i(MAX_SPEED_DEG_S), false);
    uart_puts(" deg/s, hard slew limit ");
    uart_int(round_i(MAX_SLEW_DEG_S), false);
    uart_puts(" deg/s; fault cut-out at ");
    uart_int(round_i(FAULT_ERR_DEG), false);
    uart_puts(" deg averaged error");
    uart_puts(clock_from_hsi ? "\r\nclock: 96 MHz from HSI (no ST-LINK MCO)\r\n"
                             : "\r\nclock: 96 MHz from ST-LINK 8 MHz MCO\r\n");
    uart_puts("telemetry: fb <t s> <leg> body femur tibia as sent/meas/err deg, then raw ADC\r\n");
    print_help();
    uart_puts("Press USER (blue button) or send 's' to start; again or 'x' to park and stop.\r\n");
}

/* ================================ Main ================================= */

int main(void)
{
    enum { IDLE, SWITCH_ON, TO_NEUTRAL, RUNNING, PARKING } state = IDLE;
    static segment_t seg;
    float    neutral[LEG_COUNT][JOINTS_PER_LEG];
    uint32_t step = 0, loops = 0, t_state = 0;
    int      cur_leg = first_leg();   /* leg that moves in MODE_ONE_BY_ONE */
    int      settle_idx = 0;          /* joint being switched on in SWITCH_ON */

    HAL_Init();
    servo_pins_low();
    clock_init();
    gpio_init();
    uart_init();
    pwm_init();
    adc_init();
    feedback_update();
    range_reset();
    neutral_targets(neutral);
    print_banner();

    uint32_t next_tick = HAL_GetTick();
    uint32_t next_telemetry = next_tick;
    uint32_t next_reminder = next_tick + 5000u;
    for (;;) {
        while ((int32_t)(HAL_GetTick() - next_tick) < 0)
            __WFI();
        next_tick += UPDATE_PERIOD_MS;

        uint32_t now     = HAL_GetTick();
        bool     pressed = button_pressed();
        char     c       = uart_getc();
        bool     go      = pressed || c == 's' || c == 'S';
        bool     stop    = pressed || c == 'x' || c == 'X';

        if (c == 'a' || c == 'A') {
            print_all_raw = !print_all_raw;
            uart_puts(print_all_raw ? "raw ADC of all channels: on\r\n" : "raw ADC of all channels: off\r\n");
        } else if (c == 'h' || c == 'H' || c == '?') {
            print_help();
        }

        feedback_update();
        if (c == 'r' || c == 'R') print_ranges();

        switch (state) {
        case IDLE:
            led(LED_BLUE, (now / 500u) & 1u);
            if ((int32_t)(now - next_reminder) >= 0) {
                next_reminder = now + 5000u;
                uart_puts("idle, last stop: ");
                uart_puts(last_stop);
                uart_puts("\r\n");
            }
            if (go && !MOTION_ENABLED) {
                uart_puts("start ignored: motion is disabled in this build (MOTION_ENABLED 0)\r\n");
                break;
            }
            if (go && PROGRAM == PROGRAM_LIMITS) {
                run_limits();                        /* blocking, up to a few minutes */
                next_tick = next_telemetry = HAL_GetTick();
                next_reminder = next_tick + 5000u;
                break;
            }
            if (go && PROGRAM == PROGRAM_CHECK) {
                run_check();                         /* blocking, about 25 s per servo */
                next_tick = next_telemetry = HAL_GetTick();
                next_reminder = next_tick + 5000u;
                break;
            }
            if (go) {
                bool ok = true;
                uart_puts("start: measured position (deg, from neutral):");
                for (int l = 0; l < LEG_COUNT; l++) {
                    if (!leg_selected(l)) continue;
                    for (int j = 0; j < JOINTS_PER_LEG; j++) {
                        uint8_t n = servo_of(l, j);
                        uart_puts(" ");
                        uart_puts(SERVOS[n].name);
                        uart_puts(" ");
                        uart_dec1(fb_deg[n], false);
                        uart_puts(" (");
                        uart_dec1(fb_deg[n] - SERVOS[n].neutral_deg, true);
                        uart_puts(")");
                        if (!fb_valid(n)) {
                            uart_puts(" INVALID adc ");
                            uart_int(fb_raw[n], false);
                            uart_puts(" spread ");
                            uart_int(fb_iqr[n], false);
                            ok = false;
                        }
                    }
                }
                uart_puts("\r\n");
                if (!ok) {
                    uart_puts("not starting: a pot reading is invalid (disconnected or noisy?)\r\n");
                    set_last_stop("start refused: invalid pot reading", NULL);
                    break;
                }
                memset(stats, 0, sizeof stats);
                led(LED_BLUE, false);
                led(LED_RED, false);
                led(LED_GREEN, true);
                settle_idx = 0;
                int l0 = 0, j0 = 0;
                joint_at(0, &l0, &j0);
                joint_enable(l0, j0);
                uart_puts("switch-on: each servo starts where its pot reads, one at a time\r\n");
                t_state = now;
                state   = SWITCH_ON;
            }
            break;

        case SWITCH_ON:
            /* The joint just switched on holds its measured angle for SETTLE_MS;
             * then its pot offset is learned and checked, and the next joint follows. */
            if (stop) {
                outputs_off();
                led(LED_GREEN, false);
                state = IDLE;
                uart_puts("stop requested during switch-on: pulses off\r\n");
                set_last_stop("stopped by user during switch-on", NULL);
                break;
            }
            if (now - t_state >= SETTLE_MS) {
                int l = 0, j = 0;
                joint_at(settle_idx, &l, &j);
                float off = fb_deg[servo_of(l, j)] - out_deg[l][j];
                uart_puts("  ");
                uart_puts(SERVOS[servo_of(l, j)].name);
                uart_puts(" on at ");
                uart_dec1(out_deg[l][j], false);
                uart_puts(" deg, pot offset ");
                uart_dec1(off, true);
                uart_puts(" deg\r\n");
                if (absf(off) > MAX_OFFSET_DEG) {
                    outputs_off();
                    led(LED_GREEN, false);
                    led(LED_RED, true);
                    state = IDLE;
                    uart_puts("FAULT: servo does not hold its first command -> all pulses off, not starting\r\n");
                    set_last_stop("switch-on check: servo did not hold its first command", SERVOS[servo_of(l, j)].name);
                    break;
                }
                offset_deg[l][j]   = off;
                offset_known[l][j] = true;
                if (joint_at(++settle_idx, &l, &j)) {
                    joint_enable(l, j);
                    t_state = now;
                } else {
                    segment_plan(&seg, neutral, 0, now);
                    uart_puts("easing to neutral in ");
                    uart_int((int32_t)seg.move_ms, false);
                    uart_puts(" ms\r\n");
                    state = TO_NEUTRAL;
                }
            }
            break;

        case TO_NEUTRAL:
            if (stop) {
                segment_plan(&seg, neutral, PARK_HOLD_MS, now);
                state = PARKING;
                uart_puts("stop requested: parking\r\n");
                break;
            }
            if (segment_step(&seg, now)) {
                step    = 0;
                loops   = 0;
                cur_leg = first_leg();
                if (TEST_MODE == MODE_ONE_BY_ONE) {
                    uart_puts("--- leg: ");
                    uart_puts(LEGS[cur_leg].name);
                    uart_puts(" ---\r\n");
                }
                start_keyframe(&seg, step, moving_legs(cur_leg), now);
                state = RUNNING;
            }
            break;

        case RUNNING:
            if (stop) {
                segment_plan(&seg, neutral, PARK_HOLD_MS, now);
                state = PARKING;
                uart_puts("stop requested: parking in ");
                uart_int((int32_t)seg.move_ms, false);
                uart_puts(" ms\r\n");
                break;
            }
            if (segment_step(&seg, now)) {
                if (seg.hold_ms) {
                    char label[] = "step 00";
                    label[5] = (char)('0' + (step + 1) / 10);
                    label[6] = (char)('0' + (step + 1) % 10);
                    log_settled(moving_legs(cur_leg), label);
                }
                if (++step >= SEQUENCE_LEN) {
                    bool round_done = true;
                    step = 0;
                    if (TEST_MODE == MODE_ONE_BY_ONE) {
                        int next = next_leg(cur_leg);
                        round_done = next <= cur_leg;
                        cur_leg = next;
                    }
                    if (round_done) {
                        loops++;
                        uart_puts("=== round ");
                        uart_int((int32_t)loops, false);
                        uart_puts(" done ===\r\n");
                        print_and_reset_stats("round");
                        if (SEQUENCE_REPEATS > 0 && loops >= (uint32_t)SEQUENCE_REPEATS) {
                            segment_plan(&seg, neutral, PARK_HOLD_MS, now);
                            state = PARKING;
                            uart_puts("repeat count reached: parking\r\n");
                            break;
                        }
                    }
                    if (TEST_MODE == MODE_ONE_BY_ONE) {
                        uart_puts("--- leg: ");
                        uart_puts(LEGS[cur_leg].name);
                        uart_puts(" ---\r\n");
                    }
                }
                start_keyframe(&seg, step, moving_legs(cur_leg), now);
            }
            break;

        case PARKING:
            led(LED_GREEN, (now / 100u) & 1u);
            if (segment_step(&seg, now)) {
                log_settled(TEST_LEGS, "park");
                print_and_reset_stats("since last round");
                outputs_off();
                led(LED_GREEN, false);
                state = IDLE;
                uart_puts("stopped: pulses off. Press USER or send 's' to run again.\r\n");
                set_last_stop("stopped by user (parked)", NULL);
            }
            break;
        }

        output_update();

        if (state != IDLE && fault_detected()) {
            outputs_off();
            print_and_reset_stats("until fault");
            led(LED_GREEN, false);
            led(LED_RED, true);
            state = IDLE;
            uart_puts("stopped by the fault cut-out. Check the joint, then press USER or send 's'.\r\n");
        }

        if ((int32_t)(now - next_telemetry) >= 0) {
            next_telemetry += TELEMETRY_MS;
            print_telemetry(now);
        }
    }
}

/* ======================= Interrupts and faults ========================= */

/* Stop every servo pulse at the end of its current frame, light the red LED
 * and halt. Uses registers only, so it is safe from any fault context. */
static void __attribute__((noreturn)) fail_safe_halt(void)
{
    __disable_irq();
    TIM2->CCR1 = 0; TIM2->CCR2 = 0; TIM2->CCR3 = 0; TIM2->CCR4 = 0;
    TIM3->CCR1 = 0; TIM3->CCR2 = 0; TIM3->CCR3 = 0; TIM3->CCR4 = 0;
    TIM4->CCR1 = 0; TIM4->CCR2 = 0; TIM4->CCR3 = 0; TIM4->CCR4 = 0;
    GPIOB->BSRR = LED_RED;
    for (;;) { }
}

void Error_Handler(void)      { fail_safe_halt(); }
void HardFault_Handler(void)  { fail_safe_halt(); }
void MemManage_Handler(void)  { fail_safe_halt(); }
void BusFault_Handler(void)   { fail_safe_halt(); }
void UsageFault_Handler(void) { fail_safe_halt(); }
void SysTick_Handler(void)    { HAL_IncTick(); }
