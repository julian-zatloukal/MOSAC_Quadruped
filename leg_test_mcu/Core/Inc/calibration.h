#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

#define CALIB_POINTS 20

// Per joint: [2][CALIB_POINTS] ADC sums (servo moving down, then up), each the sum of CALIB_SAMPLES_PER_POINT readings
extern const float *const CALIB_TABLES[12];
// Servo angle (deg) of each calibration point; pulse width = angle * 1000/140 + 500 us
extern const float *const CALIB_ANGLES;
extern const float CALIB_SAMPLES_PER_POINT;

#endif
