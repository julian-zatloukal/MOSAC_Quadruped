/* Hardware layer. hw_stm32.c implements it for the NUCLEO-F412ZG; everything
 * else in gait_runner only talks to the hardware through these functions.
 * (leg_lab's layer, with all 12 servos connected.) */
#ifndef GAIT_RUNNER_HW_H
#define GAIT_RUNNER_HW_H

#include <stdbool.h>
#include <stdint.h>

#define HW_ADC_SCANS    64      /* DMA ring depth, ~17 ms of scans          */
#define HW_ADC_CH       13      /* 12 pots (rank n+1 = servo n) + VREFINT   */
#define HW_ADC_VREF     12      /* channel index of VREFINT                 */

enum { HW_LED_GREEN, HW_LED_BLUE, HW_LED_RED };

/* Drives all 12 servo signal pins low first, then sets up clocks, UART, ADC and
 * the timers, every compare at 0 (no pulse). The pins are handed to the timers
 * only when connect_pwm is true; otherwise they stay driven low for good. */
void     hw_init(bool connect_pwm);

uint32_t hw_millis(void);
void     hw_wait_until(uint32_t t_ms);      /* sleeps until hw_millis() >= t_ms */

/* Pulse width of servo n from the next PWM frame on; 0 = no pulse. A pulse in
 * progress is never cut short (CCR preload). Ignored for unconnected servos. */
void     hw_set_pulse_us(int n, uint16_t us);

/* Copies the newest `count` complete samples of ADC channel ch, oldest first. */
void     hw_adc_copy(int ch, int count, uint16_t *dst);
uint16_t hw_vrefint_cal(void);              /* factory VREFINT reading at 3.3 V */

void     hw_uart_write(const char *s);      /* queued, interrupt driven */
int      hw_uart_getc(void);                /* -1 if nothing received   */

bool     hw_button_down(void);              /* USER (blue) button       */
void     hw_led(int led, bool on);
bool     hw_clock_from_hsi(void);

#endif
