#ifndef SPI_DMA_H
#define SPI_DMA_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "hardware/spi.h"
#include "hardware/dma.h"

typedef void (*spi_dma_done_cb_t)(void *user_data);

typedef struct {
    spi_inst_t       *spi;
    uint              cs_pin;
    uint              ch_tx, ch_rx;
    volatile bool     busy;
    spi_dma_done_cb_t on_done;
    void             *user_data;
} spi_dma_t;

/* One-time init — claims two DMA channels, installs DMA_IRQ_0 handler. */
void spi_dma_init(spi_dma_t *dev,
                  spi_inst_t *spi, uint cs_pin, uint baud,
                  uint pin_miso, uint pin_mosi, uint pin_sck,
                  spi_cpol_t cpol, spi_cpha_t cpha);

/* Full-duplex DMA transfer (tx or rx may be NULL for discard/dummy). */
void spi_dma_transfer(spi_dma_t *dev,
                      const uint8_t *tx, uint8_t *rx, size_t len,
                      spi_dma_done_cb_t cb, void *user_data);

/* Must be registered as the exclusive DMA_IRQ_0 handler. */
void spi_dma_irq_handler(void);

#endif
