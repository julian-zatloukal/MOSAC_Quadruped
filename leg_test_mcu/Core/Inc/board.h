#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// PWM: 96 MHz timer clock / 96 = 1 us per tick, period 3101 us (same as the main firmware)
#define PWM_PRESCALER   (96 - 1)
#define PWM_ARR         3100

// ADC feedback: continuous scan of the leg's 3 channels into a circular DMA buffer.
// 12 MHz ADC clock, 480 + 12 cycles per conversion -> 123 us per scan -> 256 scans cover ~31 ms.
#define FEEDBACK_SCANS  256

// NUCLEO-144 user LEDs and button
#define LED_GREEN_PIN   GPIO_PIN_0    // LD1, PB0
#define LED_BLUE_PIN    GPIO_PIN_7    // LD2, PB7
#define LED_RED_PIN     GPIO_PIN_14   // LD3, PB14
#define LED_PORT        GPIOB
#define BUTTON_PIN      GPIO_PIN_13   // B1 (blue), PC13, high when pressed
#define BUTTON_PORT     GPIOC

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef huart3;

void board_init(const uint8_t leg_joints[3]);
void board_start_feedback(volatile uint16_t *buffer, uint32_t length);
void board_led(uint16_t pin, bool on);
bool board_button_raw(void);
void board_release_all_pwm(void);  // safe from fault handlers
void Error_Handler(void);

#endif
