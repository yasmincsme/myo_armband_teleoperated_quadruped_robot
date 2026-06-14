#ifndef SPI_ASYNC_H
#define SPI_ASYNC_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdint.h>
#include <stdbool.h>

typedef void (*spi_done_cb_t)(void *user, uint8_t val);

void spi_async_init (spi_inst_t *spi, uint pin_miso, uint pin_mosi,
                     uint pin_sck, uint pin_cs, uint baudrate);
void spi_async_read (uint8_t reg, spi_done_cb_t cb, void *user);
void spi_async_write(uint8_t reg, uint8_t val, spi_done_cb_t cb, void *user);
bool spi_async_busy (void);

#endif
