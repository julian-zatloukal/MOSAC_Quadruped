/* Serial text output. con_printf understands %s %c %d %u %% and %f with an
 * optional '+' flag and precision 0..3 (default 1), e.g. "%+.2f". Widths are
 * ignored. No heap, no newlib printf. */
#ifndef GAIT_RUNNER_CONSOLE_H
#define GAIT_RUNNER_CONSOLE_H

void con_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void con_puts(const char *s);

#endif
