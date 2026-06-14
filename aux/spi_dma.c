#include "spi_dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

static spi_dma_t *_active_dev;
static uint8_t    _dummy_tx = 0x00;
static uint8_t    _dummy_rx;

void spi_dma_irq_handler(void) {
    spi_dma_t *dev = _active_dev;
    if (!dev) { dma_hw->ints0 = 0xFFFFFFFFu; return; }
    dma_channel_acknowledge_irq0(dev->ch_rx);
    while (spi_is_busy(dev->spi))
        tight_loop_contents();
    gpio_put(dev->cs_pin, 1);
    dev->busy = false;
    if (dev->on_done)
        dev->on_done(dev->user_data);
}

void spi_dma_init(spi_dma_t *dev, spi_inst_t *spi, uint cs_pin, uint baud,
                  uint pin_miso, uint pin_mosi, uint pin_sck,
                  spi_cpol_t cpol, spi_cpha_t cpha) {
    dev->spi    = spi;
    dev->cs_pin = cs_pin;
    dev->busy   = false;

    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);

    spi_init(spi, baud);
    spi_set_format(spi, 8, cpol, cpha, SPI_MSB_FIRST);
    gpio_set_function(pin_miso, GPIO_FUNC_SPI);
    gpio_set_function(pin_mosi, GPIO_FUNC_SPI);
    gpio_set_function(pin_sck,  GPIO_FUNC_SPI);

    dev->ch_tx = dma_claim_unused_channel(true);
    dev->ch_rx = dma_claim_unused_channel(true);

    dma_channel_set_irq0_enabled(dev->ch_rx, true);
    irq_set_exclusive_handler(DMA_IRQ_0, spi_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

void spi_dma_transfer(spi_dma_t *dev,
                      const uint8_t *tx, uint8_t *rx, size_t len,
                      spi_dma_done_cb_t cb, void *user_data) {
    dev->on_done   = cb;
    dev->user_data = user_data;
    dev->busy      = true;
    _active_dev    = dev;

    gpio_put(dev->cs_pin, 0);

    dma_channel_config tx_cfg = dma_channel_get_default_config(dev->ch_tx);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
    channel_config_set_dreq(&tx_cfg, spi_get_dreq(dev->spi, true));
    channel_config_set_read_increment(&tx_cfg, tx != NULL);
    channel_config_set_write_increment(&tx_cfg, false);
    dma_channel_configure(dev->ch_tx, &tx_cfg,
                          &spi_get_hw(dev->spi)->dr,
                          tx ? tx : &_dummy_tx, len, false);

    dma_channel_config rx_cfg = dma_channel_get_default_config(dev->ch_rx);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);
    channel_config_set_dreq(&rx_cfg, spi_get_dreq(dev->spi, false));
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, rx != NULL);
    dma_channel_configure(dev->ch_rx, &rx_cfg,
                          rx ? rx : &_dummy_rx,
                          &spi_get_hw(dev->spi)->dr, len, false);

    dma_start_channel_mask((1u << dev->ch_tx) | (1u << dev->ch_rx));
}
