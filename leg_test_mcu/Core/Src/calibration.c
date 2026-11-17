#include "calibration.h"

// Single source of truth: the tables measured for the main firmware
#include "../../../quadruped_control_mcu/Core/Inc/calibration_ADC.h"

_Static_assert(MED_CALIB == CALIB_POINTS, "calibration table size changed");

const float *const CALIB_TABLES[12] = {
  &ADC_VALUES_SERVO_0[0][0], &ADC_VALUES_SERVO_1[0][0], &ADC_VALUES_SERVO_2[0][0],  &ADC_VALUES_SERVO_3[0][0],
  &ADC_VALUES_SERVO_4[0][0], &ADC_VALUES_SERVO_5[0][0], &ADC_VALUES_SERVO_6[0][0],  &ADC_VALUES_SERVO_7[0][0],
  &ADC_VALUES_SERVO_8[0][0], &ADC_VALUES_SERVO_9[0][0], &ADC_VALUES_SERVO_10[0][0], &ADC_VALUES_SERVO_11[0][0],
};

const float *const CALIB_ANGLES = ANGLES;
const float CALIB_SAMPLES_PER_POINT = MEAS_PER_ANG;
