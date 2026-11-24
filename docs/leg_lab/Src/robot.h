/* Robot description: servo numbering, pins, legs. No hardware access here. */
#ifndef LEG_LAB_ROBOT_H
#define LEG_LAB_ROBOT_H

#include <stdint.h>

#define SERVO_COUNT 12
#define LEG_COUNT   4
#define NJ          3       /* joints per leg: body, femur, tibia */

/* Servo n is driven by TIM(2 + n/4) channel n%4 + 1 and its pot is ADC1 rank n+1,
 * as in quadruped_control_mcu and Programmed_Gait. */
typedef struct {
    const char *name;       /* BFR, FFR, ...                                        */
    const char *label;      /* sticker of the servo the calibration table was made on */
    const char *pwm_pin;
    const char *fb_pin;
    uint8_t     adc_ch;     /* ADC1 input number                                     */
    int8_t      dir;        /* servo deg = mid point + dir * q (main.py sign mapping) */
    uint8_t     leg, joint;
} servo_info_t;

typedef struct {
    const char *name;
    const char *tag;
    uint8_t     servo[NJ];  /* body, femur, tibia */
} leg_info_t;

extern const servo_info_t SERVO[SERVO_COUNT];
extern const leg_info_t   LEG[LEG_COUNT];
extern const char *const  JOINT_NAME[NJ];

/* Servo PWM scale, the same as quadruped_control_mcu. */
#define US_AT_0_DEG     500.0f
#define US_PER_DEG      (1000.0f / 140.0f)
#define PWM_PERIOD_US   3100u   /* ~322 Hz frame */

#endif
