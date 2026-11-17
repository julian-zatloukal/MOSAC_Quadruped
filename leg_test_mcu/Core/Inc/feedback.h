#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

// Slots 0..2 = body, femur, tibia of the leg under test

void feedback_init(const uint8_t leg_joints[3]);  // also starts the ADC
void feedback_update(void);                       // median of the last ~31 ms of samples, then ADC -> angle
uint16_t feedback_raw(uint8_t slot);              // filtered ADC counts
float feedback_deg(uint8_t slot);                 // servo angle (deg), same scale as the commands
bool feedback_valid(uint8_t slot);                // reading inside the calibrated range

#endif
