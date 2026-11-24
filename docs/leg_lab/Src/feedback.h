/* Pot feedback: medians of the ADC ring, conversion to degrees, validity, and the
 * electrical watch (reference pots of the unpowered legs, VDDA). */
#ifndef LEG_LAB_FEEDBACK_H
#define LEG_LAB_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>
#include "robot.h"

#define NOMINAL_ADC0      190.0f    /* every table in calibration_ADC.h: ~190 codes at 0 deg ... */
#define NOMINAL_ADC_DEG   12.62f    /* ... and 12.55..12.70 codes per degree                    */

typedef struct {
    uint16_t raw;       /* median of the ~17 ms window, as read */
    float    code;      /* the same after the common-mode correction (what deg uses) */
    uint16_t iqr;       /* spread (25 % .. 75 %) of the window */
    float    deg;       /* converted with the servo's pot model */
    bool     valid;     /* in range, steady and trusted model   */
} pot_t;

extern pot_t fb_pot[SERVO_COUNT];
extern float fb_vdda;                       /* volts, from VREFINT */

void  fb_update(void);                      /* call every tick */
float fb_fast_deg(int n);                   /* median of the newest 8 scans (~2 ms) */
float fb_fast_raw(int n);
float fb_code_to_deg(int n, float code);    /* with the servo's model */
float fb_table_deg(int n, float code);      /* with its calibration_ADC.h table, whatever the model */
float fb_table_code(int n, int k, int up);  /* table code (one reading) at ANGLES[k], up or down sweep */
float fb_angle(int k);                      /* ANGLES[k] */
int   fb_table_points(void);                /* MED_CALIB */
int   fb_meas_per_angle(void);              /* MEAS_PER_ANG */
bool  fb_in_range(int n, float code);
bool  fb_trusted(int n);
const char *fb_model_name(int n);

/* Electrical watch. fb_ref_arm() takes the valid pots that do not belong to `leg`
 * as references (those servos get no pulses, so they must stay still) and the
 * current VDDA as baseline. */
void  fb_ref_arm(int leg);
int   fb_ref_count(void);
float fb_ref_shift(void);                   /* largest reference move, deg, 17 ms medians */
float fb_ref_shift_fast(void);              /* same, ~2 ms medians */
float fb_vdda_dip(void);                    /* baseline - now, volts */
float fb_common_deg(void);                  /* common-mode correction applied now, deg */

#endif
