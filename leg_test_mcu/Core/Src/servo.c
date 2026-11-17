#include "servo.h"
#include "joints.h"

void servo_set_deg(uint8_t joint, float deg)
{
  if (deg < SERVO_MIN_DEG)
    deg = SERVO_MIN_DEG;
  else if (deg > SERVO_MAX_DEG)
    deg = SERVO_MAX_DEG;

  uint32_t ton_us = (uint32_t)(deg * SERVO_US_PER_DEG + SERVO_BIAS_US + 0.5f);
  __HAL_TIM_SET_COMPARE(JOINTS[joint].htim, JOINTS[joint].tim_channel, ton_us);
}

void servo_release(uint8_t joint)
{
  __HAL_TIM_SET_COMPARE(JOINTS[joint].htim, JOINTS[joint].tim_channel, 0);
}
