/* Pot feedback: medians of the ADC ring, conversion to degrees, validity, VDDA.
 * The same conversion as leg_lab (calibration_ADC.h tables, or a POT_LINE). */
#ifndef GAIT_RUNNER_POTS_H
#define GAIT_RUNNER_POTS_H

#include <stdbool.h>
#include <stdint.h>
#include "robot.h"

#define NOMINAL_ADC0      190.0f    /* every table in calibration_ADC.h: ~190 codes at 0 deg ... */
#define NOMINAL_ADC_DEG   12.62f    /* ... and 12.55..12.70 codes per degree                    */

typedef struct {
    uint16_t raw;       /* median of the ~17 ms window       */
    uint16_t iqr;       /* spread (25 % .. 75 %) of the window */
    float    deg;       /* converted with the servo's model   */
    bool     valid;     /* in range, steady and trusted model */
} pot_t;

extern pot_t pot[SERVO_COUNT];
extern float pot_vdda;                      /* volts, from VREFINT */

void  pots_update(void);                    /* every tick */
float pot_fast_deg(int n);                  /* median of the newest 8 scans (~2 ms) */
bool  pot_trusted(int n);
const char *pot_model_name(int n);

#endif
