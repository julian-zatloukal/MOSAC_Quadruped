#include "joints.h"

// Reference-pose offsets, as in quadruped_control_mcu/Core/Inc/main.h
#define DELTA_REF_FEMUR 42.65021f
#define DELTA_REF_TIBIA 18.34979f

/*
 * Index = joint index of the main firmware (timer TIM2..TIM4, channel 1..4 in order).
 * Home angles are the MID_POINT_* values of quadruped_control_mcu/Core/Inc/main.h: keep them in sync.
 * Signs convert the leg frame of the policy/simulation to servo direction (quadruped_control_rpi/main.py).
 * Feedback inputs follow quadruped_control_mcu/Core/Src/adc.c and the interconnection board schematic.
 */
const joint_info_t JOINTS[JOINT_COUNT] = {
  //  name   type         timer   channel        PWM pin                    pin label  ADC channel     ADC pin                home                              sign  header   terminal    input
  { "BFR", JOINT_BODY,  &htim2, TIM_CHANNEL_1, GPIOA, GPIO_PIN_15, GPIO_AF1_TIM2, "PA15", ADC_CHANNEL_4,  GPIOA, GPIO_PIN_4, 96.1285553f,                       +1, "T2_1", "J20 pin 3", "IN4"  },
  { "BBR", JOINT_BODY,  &htim2, TIM_CHANNEL_2, GPIOA, GPIO_PIN_1,  GPIO_AF1_TIM2, "PA1",  ADC_CHANNEL_13, GPIOC, GPIO_PIN_3, 80.5f,                             -1, "T2_2", "J22 pin 3", "IN13" },
  { "BBL", JOINT_BODY,  &htim2, TIM_CHANNEL_3, GPIOB, GPIO_PIN_10, GPIO_AF1_TIM2, "PB10", ADC_CHANNEL_2,  GPIOA, GPIO_PIN_2, 85.4869385f,                       -1, "T2_3", "J19 pin 1", "IN2"  },
  { "TFR", JOINT_TIBIA, &htim2, TIM_CHANNEL_4, GPIOB, GPIO_PIN_11, GPIO_AF1_TIM2, "PB11", ADC_CHANNEL_11, GPIOC, GPIO_PIN_1, 69.0105972f + DELTA_REF_TIBIA,     +1, "T2_4", "J20 pin 2", "IN11" },
  { "FFR", JOINT_FEMUR, &htim3, TIM_CHANNEL_1, GPIOC, GPIO_PIN_6,  GPIO_AF2_TIM3, "PC6",  ADC_CHANNEL_14, GPIOC, GPIO_PIN_4, 71.9034348f + DELTA_REF_FEMUR,     +1, "T3_1", "J20 pin 1", "IN14" },
  { "BFL", JOINT_BODY,  &htim3, TIM_CHANNEL_2, GPIOC, GPIO_PIN_7,  GPIO_AF2_TIM3, "PC7",  ADC_CHANNEL_7,  GPIOA, GPIO_PIN_7, 93.9473724f,                       +1, "T3_2", "J21 pin 3", "IN7"  },
  { "FFL", JOINT_FEMUR, &htim3, TIM_CHANNEL_3, GPIOC, GPIO_PIN_8,  GPIO_AF2_TIM3, "PC8",  ADC_CHANNEL_5,  GPIOA, GPIO_PIN_5, 134.817856f - DELTA_REF_FEMUR,     -1, "T3_3", "J21 pin 1", "IN5"  },
  { "TFL", JOINT_TIBIA, &htim3, TIM_CHANNEL_4, GPIOC, GPIO_PIN_9,  GPIO_AF2_TIM3, "PC9",  ADC_CHANNEL_6,  GPIOA, GPIO_PIN_6, 120.135033f - DELTA_REF_TIBIA,     -1, "T3_4", "J21 pin 2", "IN6"  },
  { "FBL", JOINT_FEMUR, &htim4, TIM_CHANNEL_1, GPIOD, GPIO_PIN_12, GPIO_AF2_TIM4, "PD12", ADC_CHANNEL_10, GPIOC, GPIO_PIN_0, 130.440994f - DELTA_REF_FEMUR,     -1, "T4_1", "J19 pin 3", "IN10" },
  { "TBL", JOINT_TIBIA, &htim4, TIM_CHANNEL_2, GPIOD, GPIO_PIN_13, GPIO_AF2_TIM4, "PD13", ADC_CHANNEL_3,  GPIOA, GPIO_PIN_3, 122.615303f - DELTA_REF_TIBIA,     -1, "T4_2", "J19 pin 2", "IN3"  },
  { "TBR", JOINT_TIBIA, &htim4, TIM_CHANNEL_3, GPIOD, GPIO_PIN_14, GPIO_AF2_TIM4, "PD14", ADC_CHANNEL_12, GPIOC, GPIO_PIN_2, 55.4315529f + DELTA_REF_TIBIA,     +1, "T4_3", "J22 pin 2", "IN12" },
  { "FBR", JOINT_FEMUR, &htim4, TIM_CHANNEL_4, GPIOD, GPIO_PIN_15, GPIO_AF2_TIM4, "PD15", ADC_CHANNEL_9,  GPIOB, GPIO_PIN_1, 55.8714142f + DELTA_REF_FEMUR,     +1, "T4_4", "J22 pin 1", "IN9"  },
};

const uint8_t LEG_JOINTS[LEG_COUNT][JOINTS_PER_LEG] = {
  [LEG_FRONT_RIGHT] = { 0, 4, 3 },    // BFR, FFR, TFR
  [LEG_FRONT_LEFT]  = { 5, 6, 7 },    // BFL, FFL, TFL
  [LEG_BACK_RIGHT]  = { 1, 11, 10 },  // BBR, FBR, TBR
  [LEG_BACK_LEFT]   = { 2, 8, 9 },    // BBL, FBL, TBL
};

const char *const LEG_NAMES[LEG_COUNT] = { "FRONT-RIGHT", "FRONT-LEFT", "BACK-RIGHT", "BACK-LEFT" };
const char *const JOINT_TYPE_NAMES[JOINTS_PER_LEG] = { "body", "femur", "tibia" };

const float JOINT_LIMIT_MIN[JOINTS_PER_LEG] = { -10.0f, -20.0f, -15.0f };
const float JOINT_LIMIT_MAX[JOINTS_PER_LEG] = {  15.0f,  30.0f,  15.0f };
