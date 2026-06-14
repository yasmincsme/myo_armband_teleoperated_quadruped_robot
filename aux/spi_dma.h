#ifndef SPI_DMA_H
#define SPI_DMA_H

#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"

typedef void (*spi_dma_done_cb_t)(void *user_data);

typedef struct {
    spi_inst_t       *spi;
    uint              cs_pin;
    int               ch_tx;
    int               ch_rx;
    volatile bool     busy;
    spi_dma_done_cb_t on_done;
    void             *user_data;
} spi_dma_t;

/* Initialize SPI peripheral and claim two DMA channels.
   Must be called once before any transfer. */
void spi_dma_init(spi_dma_t *dev,
                  spi_inst_t *spi,
                  uint cs_pin,
                  uint baud,
                  uint pin_miso,
                  uint pin_mosi,
                  uint pin_sck,
                  spi_cpol_t cpol,
                  spi_cpha_t cpha);

/* Start an async SPI transfer (full-duplex).
   tx  – bytes to send (must stay valid until on_done fires)
   rx  – buffer for received bytes (NULL → dummy sink)
   len – number of bytes
   cb  – called from DMA IRQ when transfer completes (may be NULL)
   DMA_IRQ_0 is used internally; do not claim it elsewhere. */
void spi_dma_transfer(spi_dma_t       *dev,
                      const uint8_t   *tx,
                      uint8_t         *rx,
                      size_t           len,
                      spi_dma_done_cb_t cb,
                      void            *user_data);

/* IRQ handler – install with irq_set_exclusive_handler(DMA_IRQ_0, spi_dma_irq_handler) */
void spi_dma_irq_handler(void);

#endif
