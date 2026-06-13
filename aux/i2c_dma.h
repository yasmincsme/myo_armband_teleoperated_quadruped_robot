#ifndef I2C_DMA_H
#define I2C_DMA_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"
#include "hardware/dma.h"

typedef void (*i2c_done_cb_t)(void *user);

/* One-time init — configures I2C hardware, claims two DMA channels,
   and installs DMA_IRQ_1 handler. */
void i2c_dma_init(i2c_inst_t *i2c, uint sda, uint scl, uint baud);

/* Async read: writes reg address then DMA-reads len bytes into dst.
   cb fires from DMA IRQ context when all bytes have been received.
   Maximum len is 16. */
void i2c_dma_read(uint8_t addr, uint8_t reg, uint8_t *dst, uint8_t len,
                  i2c_done_cb_t cb, void *user);

bool i2c_dma_busy(void);

#endif
