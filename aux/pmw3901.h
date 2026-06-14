#ifndef PMW3901_H
#define PMW3901_H

#include <stdint.h>
#include <stdbool.h>

/* Calibrate PMW_SCALE against a known displacement */
#define PMW_SCALE  (1.0f / 2100.0f)
#define SQUAL_MIN  20

/* SPI + control pin mapping (Pico 2) */
#define PIN_MISO   16
#define PIN_MOSI   19
#define PIN_SCK    18
#define PIN_CS     17
#define PIN_RESET  20

typedef void (*pmw3901_done_cb_t)(bool ok);

/* Start async bringup. on_done is called from IRQ context. */
void pmw3901_init    (pmw3901_done_cb_t on_done);

/* True once bringup succeeds and repeating timer is running. */
bool pmw3901_ready   (void);

/*
 * Call every main-loop iteration.
 * Starts one read chain if mot_pending && !read_chain_busy.
 */
void pmw3901_service (void);

/*
 * Atomic snapshot for the ROS timer callback.
 * Returns accumulated body-frame displacement since last call (resets to 0)
 * and most-recent instantaneous velocity.
 */
void pmw3901_snapshot(float *disp_x, float *disp_y,
                      float *vel_dx,  float *vel_dy);

#endif
