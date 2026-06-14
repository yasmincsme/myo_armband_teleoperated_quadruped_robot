#ifndef I2C_DMA_H
#define I2C_DMA_H

#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"

typedef void (*i2c_dma_done_cb_t)(void *user_data, bool ok);

typedef struct {
    i2c_inst_t        *i2c;
    int                ch_tx;
    uint8_t            addr;
    volatile bool      busy;
    i2c_dma_done_cb_t  on_done;
    void              *user_data;
    /* Internal RX state (polled via STOP_DET) */
    uint8_t           *rx_buf;
    size_t             rx_len;
    size_t             rx_idx;
} i2c_dma_t;

/* Initialize I2C peripheral and claim one DMA TX channel.
   RX bytes are collected via the STOP_DET interrupt on the I2C peripheral.
   Install i2c_dma_irq_handler on the appropriate I2C IRQ after calling this. */
void i2c_dma_init(i2c_dma_t  *dev,
                  i2c_inst_t *i2c,
                  uint        baud,
                  uint        pin_sda,
                  uint        pin_scl);

/* Async write: addr + reg + data[0..len-1].  on_done fires from IRQ. */
void i2c_dma_write(i2c_dma_t        *dev,
                   uint8_t           addr,
                   const uint8_t    *data,
                   size_t            len,
                   i2c_dma_done_cb_t cb,
                   void             *user_data);

/* Async write-then-read: sends reg, repeated-start, reads rx_len bytes.
   on_done fires from IRQ when all rx_len bytes have been received. */
void i2c_dma_write_read(i2c_dma_t        *dev,
                         uint8_t           addr,
                         uint8_t           reg,
                         uint8_t          *rx_buf,
                         size_t            rx_len,
                         i2c_dma_done_cb_t cb,
                         void             *user_data);

/* IRQ handler – install with irq_set_exclusive_handler(I2C0_IRQ, ...) or I2C1_IRQ */
void i2c_dma_irq_handler(i2c_dma_t *dev);

/* Blocking helpers for use during init (before async reads begin) */
bool i2c_dma_write_reg_blocking(i2c_dma_t *dev, uint8_t addr, uint8_t reg, uint8_t val);
bool i2c_dma_read_bytes_blocking(i2c_dma_t *dev, uint8_t addr, uint8_t reg,
                                  uint8_t *buf, size_t len);

#endif
