#ifndef LOG_H
#define LOG_H

// Non-blocking text output on USART3 (ST-LINK virtual COM port). Drops characters if the buffer is full.
void log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void log_uart_irq(void);

#define LOG_DEG_LEN 16

// Formats v with one decimal and a sign ("+12.3", "-0.4") into out[LOG_DEG_LEN]; returns out
const char *log_deg(char *out, float v);

#endif
