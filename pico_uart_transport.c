#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <uxr/client/profile/transport/custom/custom_transport.h>

#define ROS_UART_ID   uart1
#define ROS_BAUD_RATE 115200
#define ROS_TX_PIN    8
#define ROS_RX_PIN    9

static uart_inst_t *ros_uart = ROS_UART_ID;

void usleep(uint64_t us) { sleep_us(us); }

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec  = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

static int uart_getc_timeout(uart_inst_t *uart, int64_t timeout_us)
{
    uint64_t start = time_us_64();
    while (!uart_is_readable(uart)) {
        if ((int64_t)(time_us_64() - start) > timeout_us)
            return -1;
    }
    return uart_getc(uart);
}

bool pico_serial_transport_open(struct uxrCustomTransport *transport)
{
    static bool require_init = true;
    if (require_init) {
        uart_init(ros_uart, ROS_BAUD_RATE);
        gpio_set_function(ROS_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(ROS_RX_PIN, GPIO_FUNC_UART);
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
    for (size_t i = 0; i < len; i++)
        uart_putc_raw(ros_uart, buf[i]);
    return len;
}

size_t pico_serial_transport_read(struct uxrCustomTransport *transport,
                                  uint8_t *buf, size_t len,
                                  int timeout, uint8_t *errcode)
{
    uint64_t start = time_us_64();
    for (size_t i = 0; i < len; i++) {
        int64_t remaining = (int64_t)timeout * 1000 - (int64_t)(time_us_64() - start);
        if (remaining < 0) { *errcode = 1; return i; }

        int c = uart_getc_timeout(ros_uart, remaining);
        if (c < 0) { *errcode = 1; return i; }
        buf[i] = (uint8_t)c;
    }
    return len;
}
