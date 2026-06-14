#ifndef PMW3901_H
#define PMW3901_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "spi_dma.h"

#define PMW_SCALE  0.001f
#define DT         0.05f

#define PIN_MISO      16
#define PIN_MOSI      19
#define PIN_SCK       18
#define PIN_CS        17
#define INTERFACE_SPI spi0

/* PMW3901 registers */
#define PMW_REG_MOTION      0x02
#define PMW_REG_DELTA_X_L   0x03
#define PMW_REG_DELTA_X_H   0x04
#define PMW_REG_DELTA_Y_L   0x05
#define PMW_REG_DELTA_Y_H   0x06
#define PMW_REG_SQUAL       0x07
#define PMW_REG_POWER_UP    0x3A
#define PMW_REG_SHUTDOWN    0x3B
#define PMW_REG_OBSERVATION 0x15

typedef struct {
    int16_t dx;
    int16_t dy;
    uint8_t motion;
    uint8_t squal;
    float   dx_m;
    float   dy_m;
} pmw3901_t;

typedef void (*pmw3901_done_cb_t)(void);

/* Initialize the SPI DMA engine and start the non-blocking register sequence.
   on_done is called (from alarm context) when the sensor is ready. */
void pmw3901_init(pmw3901_done_cb_t on_done);

/* Call repeatedly in main loop until pmw3901_init_done() returns true. */
void pmw3901_init_poll(void);
bool pmw3901_init_done(void);

/* Start an async burst read; on_done fires (from DMA IRQ) when data is ready.
   Call pmw3901_get_values() inside on_done to retrieve results. */
void pmw3901_read_async(pmw3901_t *out, pmw3901_done_cb_t on_done);

/* Blocking single-register read (for init use only). */
uint8_t pmw3901_read_reg(uint8_t reg);

#endif
