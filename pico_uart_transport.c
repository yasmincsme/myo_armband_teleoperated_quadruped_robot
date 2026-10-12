#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include <uxr/client/profile/transport/custom/custom_transport.h>

/* micro-ROS agent link (host comms) — USB CDC, to the Raspberry Pi that
   powers the Pico and hosts the agent. The SSC-32 servo controller uses
   uart1/GPIO 8-9 separately (see SSC_UART in pico_micro_ros_example.c). */

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec  = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

bool pico_serial_transport_open(struct uxrCustomTransport *transport)
{
    static bool require_init = true;
    if (require_init) {
        stdio_init_all();
        require_init = false;
    }
    return true;
}

bool pico_serial_transport_close(struct uxrCustomTransport *transport)
{
    return true;
}

size_t pico_serial_transport_write(struct uxrCustomTransport *transport,
                                   const uint8_t *buf, size_t len,
                                   uint8_t *errcode)
{
    for (size_t i = 0; i < len; i++) {
        if (buf[i] != (uint8_t)putchar_raw(buf[i])) {
            *errcode = 1;
            return i;
        }
    }
    return len;
}

size_t pico_serial_transport_read(struct uxrCustomTransport *transport,
                                  uint8_t *buf, size_t len,
                                  int timeout, uint8_t *errcode)
{
    uint64_t start_time_us = time_us_64();
    for (size_t i = 0; i < len; i++) {
        int64_t elapsed = (int64_t)(timeout) * 1000
                        - (int64_t)(time_us_64() - start_time_us);
        if (elapsed < 0) {
            *errcode = 1;
            return i;
        }
        int c = getchar_timeout_us((uint32_t)elapsed);
        if (c == PICO_ERROR_TIMEOUT) {
            *errcode = 1;
            return i;
        }
        buf[i] = (uint8_t)c;
    }
    return len;
}
