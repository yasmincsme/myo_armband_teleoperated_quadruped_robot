#ifndef HCSR04_H
#define HCSR04_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define HCSR04_TRIG_PIN 12
#define HCSR04_ECHO_PIN 13

typedef struct {
    volatile uint64_t start_time;
    volatile uint64_t end_time;
    volatile bool measure_done;
    volatile float distance;
} hcsr04_t;

void hcsr04_init(hcsr04_t *sensor);

#endif
