#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

// Servo angle <-> pulse width, as in the main firmware and the calibration: 0 deg = 500 us, 140 deg per 1000 us
#define SERVO_US_PER_DEG  (1000.0f / 140.0f)
#define SERVO_BIAS_US     500.0f
#define SERVO_MIN_DEG     0.0f
#define SERVO_MAX_DEG     266.0f   // last calibration point (2400 us)

void servo_set_deg(uint8_t joint, float deg);  // clamps to the calibrated range
void servo_release(uint8_t joint);             // no pulses: the servo goes limp

#endif
