#include "pots.h"
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

pot_t pot[SERVO_COUNT];
float pot_vdda = 3.3f;

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

static float mean_code(int n, int k) { return (TABLE[n][0][k] + TABLE[n][1][k]) * 0.5f / MEAS_PER_ANG; }

/* Linear interpolation over the mean of the down and up tables, clipped at the ends. */
static float table_deg(int n, float code)
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

static float code_to_deg(int n, float code)
{
    switch (MODEL[n]) {
    case POT_LINE:      return LINE[n][1] > 1.0f ? (code - LINE[n][0]) / LINE[n][1] : 0.0f;
    case POT_UNTRUSTED: return (code - NOMINAL_ADC0) / NOMINAL_ADC_DEG;    /* display only */
    default:            return table_deg(n, code);
    }
}

static bool in_range(int n, float code)
{
    switch (MODEL[n]) {
    case POT_LINE: {
        float d = code_to_deg(n, code);
        return LINE[n][1] > 1.0f && d > 0.0f && d < (float)MAX_TABLE_ANGLE;
    }
    case POT_UNTRUSTED:
        return code > 50.0f && code < 4045.0f;
    default:
        return code > mean_code(n, 0) && code < mean_code(n, MED_CALIB - 1);
    }
}

bool pot_trusted(int n) { return MODEL[n] != POT_UNTRUSTED; }

const char *pot_model_name(int n)
{
    switch (MODEL[n]) {
    case POT_LINE:      return "line";
    case POT_UNTRUSTED: return "UNTRUSTED";
    default:            return "table";
    }
}

void pots_update(void)
{
    for (int n = 0; n < SERVO_COUNT; n++) {
        uint16_t iqr;
        float raw = median_of(n, HW_ADC_SCANS, &iqr);
        pot[n].raw   = (uint16_t)(raw + 0.5f);
        pot[n].iqr   = iqr;
        pot[n].deg   = code_to_deg(n, raw);
        pot[n].valid = iqr <= FB_MAX_IQR && in_range(n, raw) && pot_trusted(n);
    }
    float vref = median_of(HW_ADC_VREF, HW_ADC_SCANS, 0);
    if (vref > 100.0f) pot_vdda = 3.3f * (float)hw_vrefint_cal() / vref;
}

float pot_fast_deg(int n) { return code_to_deg(n, median_of(n, FAST_SCANS, 0)); }
