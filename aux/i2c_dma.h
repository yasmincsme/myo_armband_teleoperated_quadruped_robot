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

/*
 * True when a transfer has been in flight longer than timeout_us. A slave that
 * never ACKs leaves the RX DMA unfinished, so the completion IRQ never fires
 * and _busy would otherwise stay set forever, permanently stalling the chain.
 */
bool i2c_dma_stalled(uint32_t timeout_us);

/* Abort an in-flight transfer and return the peripheral to a usable state. */
void i2c_dma_abort(void);

/* Diagnostic: how many transfers have been aborted by the stall watchdog. */
extern volatile uint32_t i2c_dma_abort_count;

/*
 * Bounded spin-wait for the STOP condition. Returns false on timeout. Called
 * from IRQ context between chained reads, so it must never spin unbounded.
 */
bool i2c_dma_wait_idle(void);

/*
 * Non-blocking I2C read: write `reg` to slave `addr`, repeated-start, read `len` bytes.
 * `cb` is called from DMA_IRQ_1 context when the last byte arrives.
 * Caller must check `!i2c_dma_busy()` before calling.
 */
void i2c_dma_read(uint8_t addr, uint8_t reg,
                  uint8_t *dst, size_t len,
                  i2c_dma_done_cb_t cb, void *user);

#endif
