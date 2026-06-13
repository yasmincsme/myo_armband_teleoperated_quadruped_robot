#ifndef PMW3901_H
#define PMW3901_H

#include <stdint.h>
#include <stdbool.h>

#define SENSOR_HEIGHT_M  2.2f
#define PMW_SCALE        (SENSOR_HEIGHT_M * 0.01f)
#define DT               0.05f

/* Pinout (Pico 2) */
#define PIN_MISO      16
#define PIN_MOSI      19
#define PIN_SCK       18
#define PIN_CS        17
#define INTERFACE_SPI spi0
#define SPI_HZ        2000000

typedef void (*pmw3901_done_cb_t)(void);

typedef struct {
    uint8_t motion;
    int16_t dx, dy;
    float   dx_m, dy_m;
} pmw3901_t;

/* Blocking phase-1 register writes (~15 ms) followed by a 100 ms async alarm. */
void pmw3901_init(pmw3901_done_cb_t on_done);

/* Call from main loop; performs phase-2 writes when the alarm has fired. */
void pmw3901_init_poll(void);

/* Returns true once both init phases are complete. */
bool pmw3901_init_done(void);

/* Non-blocking burst read of MOTION + DX + DY into *out.
   on_done (optional) is called from IRQ context when data is ready. */
void pmw3901_read_async(pmw3901_t *out, pmw3901_done_cb_t on_done);

/* Blocking register read (for diagnostics; call only before first read_async). */
uint8_t pmw3901_read_reg(uint8_t reg);

#endif
