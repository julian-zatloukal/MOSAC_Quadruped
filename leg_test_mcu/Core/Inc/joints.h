#ifndef JOINTS_H
#define JOINTS_H

#include "stm32f4xx_hal.h"

#define JOINT_COUNT     12
#define JOINTS_PER_LEG  3

typedef enum { JOINT_BODY = 0, JOINT_FEMUR = 1, JOINT_TIBIA = 2 } joint_type_t;

typedef enum { LEG_FRONT_RIGHT = 0, LEG_FRONT_LEFT, LEG_BACK_RIGHT, LEG_BACK_LEFT, LEG_COUNT } leg_id_t;

typedef struct {
  const char        *name;          // joint code used in the main firmware: BFR, FFR, TFR, ...
  joint_type_t       type;
  TIM_HandleTypeDef *htim;          // PWM timer
  uint32_t           tim_channel;   // TIM_CHANNEL_x
  GPIO_TypeDef      *pwm_port;
  uint16_t           pwm_pin;
  uint8_t            pwm_af;
  const char        *pwm_label;     // MCU pin, for the log
  uint32_t           adc_channel;   // ADC_CHANNEL_x of the potentiometer feedback
  GPIO_TypeDef      *adc_port;
  uint16_t           adc_pin;
  float              home_deg;      // servo angle at the reference pose (main firmware mid point)
  int8_t             sign;          // servo offset = sign * leg-frame offset (same convention as the policy)
  const char        *pcb_pwm;       // PWM header silkscreen on the interconnection board
  const char        *pcb_feedback;  // feedback terminal block and pin
  const char        *adc_label;     // ADC input label (INx), also printed next to the terminal
} joint_info_t;

extern const joint_info_t JOINTS[JOINT_COUNT];

// Joint indexes of each leg, ordered body, femur, tibia
extern const uint8_t LEG_JOINTS[LEG_COUNT][JOINTS_PER_LEG];
extern const char *const LEG_NAMES[LEG_COUNT];
extern const char *const JOINT_TYPE_NAMES[JOINTS_PER_LEG];

// Allowed range around home, in the leg frame (deg), indexed by joint_type_t.
// Same limits as the main firmware (BODY/FEMUR/TIBIA_RANGE_* in quadruped_control_mcu main.h).
extern const float JOINT_LIMIT_MIN[JOINTS_PER_LEG];
extern const float JOINT_LIMIT_MAX[JOINTS_PER_LEG];

// Timer handles, defined in board.c
extern TIM_HandleTypeDef htim2, htim3, htim4;

#endif
