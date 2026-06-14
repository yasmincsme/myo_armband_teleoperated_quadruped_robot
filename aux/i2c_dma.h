#ifndef I2C_DMA_H
#define I2C_DMA_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef void (*i2c_dma_done_cb_t)(void *user);

/* Call once after i2c_activate(). Uses DMA_IRQ_1 (no conflict with SPI on DMA_IRQ_0). */
void i2c_dma_init(i2c_inst_t *i2c);

bool i2c_dma_busy(void);

/* Spin-wait for STOP condition to finish. Safe to call from IRQ between chained reads. */
void i2c_dma_wait_idle(void);

/*
 * Non-blocking I2C read: write `reg` to slave `addr`, repeated-start, read `len` bytes.
 * `cb` is called from DMA_IRQ_1 context when the last byte arrives.
 * Caller must check `!i2c_dma_busy()` before calling.
 */
void i2c_dma_read(uint8_t addr, uint8_t reg,
                  uint8_t *dst, size_t len,
                  i2c_dma_done_cb_t cb, void *user);

#endif
