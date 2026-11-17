#include "board.h"
#include "log.h"

static void fault_stop(void)
{
  __disable_irq();
  board_release_all_pwm();
  LED_PORT->BSRR = LED_RED_PIN;
  while (1)
  {
  }
}

void NMI_Handler(void) { fault_stop(); }
void HardFault_Handler(void) { fault_stop(); }
void MemManage_Handler(void) { fault_stop(); }
void BusFault_Handler(void) { fault_stop(); }
void UsageFault_Handler(void) { fault_stop(); }

void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void DMA2_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void USART3_IRQHandler(void)
{
  log_uart_irq();
}
