#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

void *memcpy(void *dest, const void *src, size_t n) {
    uint8_t *d = dest;
    const uint8_t *s = src;
    while (n--) *d++ = *s++;
    return dest;
}

void *memmove(void *dest, const void *src, size_t n) {
    uint8_t *d = dest;
    const uint8_t *s = src;
    if (d < s) {
        while (n--) *d++ = *s++;
    } else {
        d += n;
        s += n;
        while (n--) *--d = *--s;
    }
    return dest;
}

int memcmp(const void *s1, const void *s2, size_t n) {
    const uint8_t *a = s1;
    const uint8_t *b = s2;
    while (n--) {
        if (*a != *b)
            return *a - *b;
        a++; b++;
    }
    return 0;
}

size_t strlen(const char *s) {
    const char *p = s;
    while (*p) ++p;
    return p - s;
}

int strcmp(const char *s1, const char *s2) {
    while (*s1 && (*s1 == *s2)) {
        ++s1;
        ++s2;
    }
    return *(unsigned char *)s1 - *(unsigned char *)s2;
}

char *strstr(const char *haystack, const char *needle) {
    if (!*needle) return (char *)haystack;

    for (; *haystack; ++haystack) {
        if ((*haystack == *needle) && !memcmp(haystack, needle, strlen(needle)))
            return (char *)haystack;
    }
    return NULL;
}

static int _mini_itoa(int value, char *buf, int base, int is_signed) {
    char tmp[16];
    int i = 0, j = 0;
    unsigned int v = (is_signed && value < 0) ? -value : value;

    do {
        tmp[i++] = "0123456789abcdef"[v % base];
        v /= base;
    } while (v && i < (int)sizeof(tmp));

    if (is_signed && value < 0)
        buf[j++] = '-';

    while (i--) buf[j++] = tmp[i];
    buf[j] = 0;
    return j;
}

int _crypto_sniprintf(char *buf, size_t size, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    size_t len = 0;
    const char *p = fmt;
    char numbuf[16];

    while (*p && len < size - 1) {
        if (*p == '%') {
            p++;
            switch (*p++) {
                case 's': {
                    const char *s = va_arg(args, const char *);
                    while (*s && len < size - 1)
                        buf[len++] = *s++;
                    break;
                }
                case 'c': {
                    char c = (char)va_arg(args, int);
                    buf[len++] = c;
                    break;
                }
                case 'd': {
                    int v = va_arg(args, int);
                    _mini_itoa(v, numbuf, 10, 1);
                    for (char *n = numbuf; *n && len < size - 1; ++n)
                        buf[len++] = *n;
                    break;
                }
                case 'u': {
                    unsigned int v = va_arg(args, unsigned int);
                    _mini_itoa(v, numbuf, 10, 0);
                    for (char *n = numbuf; *n && len < size - 1; ++n)
                        buf[len++] = *n;
                    break;
                }
                case 'x': {
                    unsigned int v = va_arg(args, unsigned int);
                    _mini_itoa(v, numbuf, 16, 0);
                    for (char *n = numbuf; *n && len < size - 1; ++n)
                        buf[len++] = *n;
                    break;
                }
                default:
                    buf[len++] = '?';
            }
        } else {
            buf[len++] = *p++;
        }
    }

    buf[len] = '\0';
    va_end(args);
    return (int)len;
}

void *_crypto_memset(void *s, int c, size_t n) {
    uint8_t *p = (uint8_t *)s;
    while (n--) *p++ = (uint8_t)c;
    return s;
}
void mbedtls_platform_zeroize(void *buf, size_t len)
{
    _crypto_memset(buf, 0, len);
}
