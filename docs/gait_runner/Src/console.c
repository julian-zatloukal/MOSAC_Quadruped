#include "console.h"
#include "hw.h"
#include <stdarg.h>
#include <stdint.h>

void con_puts(const char *s) { hw_uart_write(s); }

static char *put_uint(char *p, uint32_t u)
{
    char tmp[11];
    int i = 0;
    do { tmp[i++] = (char)('0' + u % 10u); u /= 10u; } while (u);
    while (i) *p++ = tmp[--i];
    return p;
}

static char *put_float(char *p, float v, int prec, int plus)
{
    static const uint32_t SCALE[4] = { 1u, 10u, 100u, 1000u };
    if (v != v) { *p++ = 'n'; *p++ = 'a'; *p++ = 'n'; return p; }
    if (v > 1e9f || v < -1e9f) { *p++ = v < 0 ? '-' : '+'; *p++ = 'i'; *p++ = 'n'; *p++ = 'f'; return p; }
    uint32_t s = SCALE[prec];
    float    a = v < 0.0f ? -v : v;
    uint32_t t = (uint32_t)(a * (float)s + 0.5f);
    if (v < 0.0f && t != 0u) *p++ = '-';
    else if (plus)           *p++ = '+';
    p = put_uint(p, t / s);
    if (prec) {
        uint32_t f = t % s;
        *p++ = '.';
        for (int k = prec - 1; k >= 0; k--) {
            *p++ = (char)('0' + (f / SCALE[k]) % 10u);
        }
    }
    return p;
}

void con_printf(const char *fmt, ...)
{
    char    buf[200];
    char   *p   = buf;
    char   *end = buf + sizeof buf - 24;    /* room for one converted value */
    va_list ap;

    va_start(ap, fmt);
    while (*fmt) {
        if (p >= end) { *p = '\0'; hw_uart_write(buf); p = buf; }
        if (*fmt != '%') { *p++ = *fmt++; continue; }
        fmt++;
        int plus = 0, prec = 1;
        while (*fmt == '+' || *fmt == '-' || *fmt == ' ' || *fmt == '0') { if (*fmt == '+') plus = 1; fmt++; }
        while (*fmt >= '0' && *fmt <= '9') fmt++;                     /* width: ignored */
        if (*fmt == '.') {
            fmt++;
            prec = 0;
            while (*fmt >= '0' && *fmt <= '9') prec = prec * 10 + (*fmt++ - '0');
            if (prec > 3) prec = 3;
        }
        while (*fmt == 'l' || *fmt == 'h') fmt++;
        switch (*fmt) {
        case 's': {
            const char *s = va_arg(ap, const char *);
            if (!s) s = "(null)";
            while (*s) {
                if (p >= end) { *p = '\0'; hw_uart_write(buf); p = buf; }
                *p++ = *s++;
            }
            break;
        }
        case 'c': *p++ = (char)va_arg(ap, int); break;
        case 'd': {
            int v = va_arg(ap, int);
            if (v < 0) { *p++ = '-'; p = put_uint(p, 0u - (uint32_t)v); }
            else { if (plus) *p++ = '+'; p = put_uint(p, (uint32_t)v); }
            break;
        }
        case 'u': p = put_uint(p, va_arg(ap, unsigned)); break;
        case 'f': p = put_float(p, (float)va_arg(ap, double), prec, plus); break;
        case '%': *p++ = '%'; break;
        case '\0': fmt--; break;
        default:  *p++ = '?'; break;
        }
        fmt++;
    }
    va_end(ap);
    *p = '\0';
    hw_uart_write(buf);
}
