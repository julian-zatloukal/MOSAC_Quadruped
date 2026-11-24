#include "feedback.h"
#include "config.h"
#include "hw.h"
#include "calibration_ADC.h"    /* defines its arrays: include it in this file only */

#define FAST_SCANS 8

static const float (*const TABLE[SERVO_COUNT])[MED_CALIB] = {
    ADC_VALUES_SERVO_0, ADC_VALUES_SERVO_1, ADC_VALUES_SERVO_2,  ADC_VALUES_SERVO_3,
    ADC_VALUES_SERVO_4, ADC_VALUES_SERVO_5, ADC_VALUES_SERVO_6,  ADC_VALUES_SERVO_7,
    ADC_VALUES_SERVO_8, ADC_VALUES_SERVO_9, ADC_VALUES_SERVO_10, ADC_VALUES_SERVO_11,
};
static const uint8_t MODEL[SERVO_COUNT]   = CFG_POT_MODEL;
static const float   LINE[SERVO_COUNT][2] = CFG_POT_LINE;

pot_t fb_pot[SERVO_COUNT];
float fb_vdda = 3.3f;

static bool  ref_on[SERVO_COUNT];
static float ref_base[SERVO_COUNT];
static int   ref_n;
static float vdda_base = 3.3f;

/* --------------------------------------------------------------- medians --- */

static void sort_u16(uint16_t *v, int n)
{
    for (int i = 1; i < n; i++) {
        uint16_t x = v[i];
        int k = i - 1;
        while (k >= 0 && v[k] > x) { v[k + 1] = v[k]; k--; }
        v[k + 1] = x;
    }
}

static float median_of(int ch, int count, uint16_t *iqr)
{
    uint16_t v[HW_ADC_SCANS];
    hw_adc_copy(ch, count, v);
    sort_u16(v, count);
    if (iqr) *iqr = (uint16_t)(v[count * 3 / 4] - v[count / 4]);
    return (v[count / 2 - 1] + v[count / 2]) * 0.5f;
}

/* ------------------------------------------------------------ conversion --- */

float fb_angle(int k)          { return ANGLES[k]; }
int   fb_table_points(void)    { return MED_CALIB; }
int   fb_meas_per_angle(void)  { return MEAS_PER_ANG; }

float fb_table_code(int n, int k, int up) { return TABLE[n][up ? 1 : 0][k] / MEAS_PER_ANG; }

static float mean_code(int n, int k) { return (TABLE[n][0][k] + TABLE[n][1][k]) * 0.5f / MEAS_PER_ANG; }

/* Linear interpolation over the mean of the down and up tables, clipped at the ends. */
float fb_table_deg(int n, float code)
{
    float prev_c = 0.0f, prev_d = 0.0f;
    for (int k = 0; k < MED_CALIB; k++) {
        float c = mean_code(n, k);
        if (code <= c) {
            if (k == 0) return ANGLES[0];
            return prev_d + (ANGLES[k] - prev_d) * (code - prev_c) / (c - prev_c);
        }
        prev_c = c;
        prev_d = ANGLES[k];
    }
    return MAX_TABLE_ANGLE;
}

float fb_code_to_deg(int n, float code)
{
    switch (MODEL[n]) {
    case POT_LINE:      return LINE[n][1] > 1.0f ? (code - LINE[n][0]) / LINE[n][1] : 0.0f;
    case POT_UNTRUSTED: return (code - NOMINAL_ADC0) / NOMINAL_ADC_DEG;    /* display only */
    default:            return fb_table_deg(n, code);
    }
}

bool fb_in_range(int n, float code)
{
    switch (MODEL[n]) {
    case POT_LINE: {
        float d = fb_code_to_deg(n, code);
        return LINE[n][1] > 1.0f && d > 0.0f && d < (float)MAX_TABLE_ANGLE;
    }
    case POT_UNTRUSTED:
        return code > 50.0f && code < 4045.0f;     /* at least not at a rail */
    default:
        return code > mean_code(n, 0) && code < mean_code(n, MED_CALIB - 1);
    }
}

bool fb_trusted(int n) { return MODEL[n] != POT_UNTRUSTED; }

const char *fb_model_name(int n)
{
    switch (MODEL[n]) {
    case POT_LINE:      return "line";
    case POT_UNTRUSTED: return "UNTRUSTED";
    default:            return "table";
    }
}

/* ---------------------------------------------------------------- update --- */

static float common_codes;

void fb_update(void)
{
    float raw[SERVO_COUNT];
    for (int n = 0; n < SERVO_COUNT; n++) {
        uint16_t iqr;
        raw[n] = median_of(n, HW_ADC_SCANS, &iqr);
        fb_pot[n].raw = (uint16_t)(raw[n] + 0.5f);
        fb_pot[n].iqr = iqr;
    }

    /* Common mode: servo current lifts or drops the ground of every pot. The pots
     * of the unpowered legs (references) cannot move, so their median shift is
     * that ground shift; it is taken off the other pots. Limited to the pause
     * threshold: beyond it the motion is paused anyway. */
    common_codes = 0.0f;
#if FB_COMMON_MODE
    if (ref_n > 0) {
        float d[SERVO_COUNT];
        int   m = 0;
        for (int n = 0; n < SERVO_COUNT; n++)
            if (ref_on[n]) d[m++] = raw[n] - ref_base[n];
        for (int i = 1; i < m; i++) {               /* insertion sort */
            float x = d[i];
            int k = i - 1;
            while (k >= 0 && d[k] > x) { d[k + 1] = d[k]; k--; }
            d[k + 1] = x;
        }
        float med = (m & 1) ? d[m / 2] : 0.5f * (d[m / 2 - 1] + d[m / 2]);
        float lim = REF_PAUSE_DEG * NOMINAL_ADC_DEG;
        common_codes = med > lim ? lim : (med < -lim ? -lim : med);
    }
#endif
    for (int n = 0; n < SERVO_COUNT; n++) {
        float c = ref_on[n] ? raw[n] : raw[n] - common_codes;
        fb_pot[n].code  = c;
        fb_pot[n].deg   = fb_code_to_deg(n, c);
        fb_pot[n].valid = fb_pot[n].iqr <= FB_MAX_IQR && fb_in_range(n, c) && fb_trusted(n);
    }
    float vref = median_of(HW_ADC_VREF, HW_ADC_SCANS, 0);
    if (vref > 100.0f) fb_vdda = 3.3f * (float)hw_vrefint_cal() / vref;
}

float fb_common_deg(void) { return common_codes / NOMINAL_ADC_DEG; }

float fb_fast_raw(int n) { return median_of(n, FAST_SCANS, 0); }
float fb_fast_deg(int n) { return fb_code_to_deg(n, fb_fast_raw(n)); }

/* ------------------------------------------------------- electrical watch --- */

void fb_ref_arm(int leg)
{
    ref_n = 0;
    for (int n = 0; n < SERVO_COUNT; n++) {
        ref_on[n] = SERVO[n].leg != leg && fb_pot[n].iqr <= FB_MAX_IQR &&
                    fb_pot[n].raw > 50u && fb_pot[n].raw < 4045u;
        ref_base[n] = fb_pot[n].raw;
        if (ref_on[n]) ref_n++;
    }
    vdda_base = fb_vdda;
}

int fb_ref_count(void) { return ref_n; }

static float ref_shift_with(bool fast)
{
    float worst = 0.0f;
    for (int n = 0; n < SERVO_COUNT; n++) {
        if (!ref_on[n]) continue;
        float raw = fast ? fb_fast_raw(n) : (float)fb_pot[n].raw;
        float d = (raw - ref_base[n]) / NOMINAL_ADC_DEG;
        if (d < 0.0f) d = -d;
        if (d > worst) worst = d;
    }
    return worst;
}

float fb_ref_shift(void)      { return ref_shift_with(false); }
float fb_ref_shift_fast(void) { return ref_shift_with(true); }
float fb_vdda_dip(void)       { return vdda_base - fb_vdda; }
