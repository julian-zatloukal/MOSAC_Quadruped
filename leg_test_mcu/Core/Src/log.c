#include "log.h"
#include "board.h"
#include <stdarg.h>
#include <stdio.h>

#define LOG_BUFFER_SIZE 4096

static char buffer[LOG_BUFFER_SIZE];
static volatile uint32_t head;  // written by log_printf
static volatile uint32_t tail;  // written by the UART interrupt

void log_printf(const char *fmt, ...)
{
  char line[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof line, fmt, ap);
  va_end(ap);
  if (n <= 0)
    return;
  if (n >= (int)sizeof line)
    n = sizeof line - 1;

  for (int i = 0; i < n; i++)
  {
    uint32_t next = (head + 1) % LOG_BUFFER_SIZE;
    if (next == tail)
      break;
    buffer[head] = line[i];
    head = next;
  }

  __disable_irq();
  USART3->CR1 |= USART_CR1_TXEIE;
  __enable_irq();
}

void log_uart_irq(void)
{
  if ((USART3->SR & USART_SR_TXE) && (USART3->CR1 & USART_CR1_TXEIE))
  {
    if (tail == head)
    {
      USART3->CR1 &= ~USART_CR1_TXEIE;
    }
    else
    {
      USART3->DR = (uint8_t)buffer[tail];
      tail = (tail + 1) % LOG_BUFFER_SIZE;
    }
  }
}

const char *log_deg(char *out, float v)
{
  int32_t tenths = (int32_t)(v * 10.0f + (v >= 0.0f ? 0.5f : -0.5f));
  char sign = tenths < 0 ? '-' : '+';
  if (tenths < 0)
    tenths = -tenths;
  snprintf(out, LOG_DEG_LEN, "%c%d.%d", sign, (int)(tenths / 10), (int)(tenths % 10));
  return out;
}
