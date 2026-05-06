#include "hcsr04.h"
#include "pico/time.h"

static hcsr04_t *_sensor = NULL;
static struct repeating_timer _trig_timer;

static void gpio_irq_handler(uint gpio, uint32_t events) {
    if (_sensor == NULL) return;

    if (gpio == HCSR04_ECHO_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
        _sensor->start_time = to_us_since_boot(get_absolute_time());
    } else if (gpio == HCSR04_ECHO_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        _sensor->end_time = to_us_since_boot(get_absolute_time());
        uint64_t duration = _sensor->end_time - _sensor->start_time;
        _sensor->distance = (duration * 0.0343f) / 2.0f;
        _sensor->measure_done = true;
    }
}

static int64_t trig_off_callback(alarm_id_t id, void *user_data) {
    (void) id;
    (void) user_data;
    gpio_put(HCSR04_TRIG_PIN, 0);
    return 0;
}

static bool trig_on_callback(struct repeating_timer *t) {
    (void) t;
    gpio_put(HCSR04_TRIG_PIN, 1);
    add_alarm_in_us(10, trig_off_callback, NULL, false);
    return true;
}

void hcsr04_init(hcsr04_t *sensor) {
    _sensor = sensor;

    gpio_init(HCSR04_TRIG_PIN);
    gpio_set_dir(HCSR04_TRIG_PIN, GPIO_OUT);
    gpio_put(HCSR04_TRIG_PIN, 0);

    gpio_init(HCSR04_ECHO_PIN);
    gpio_set_dir(HCSR04_ECHO_PIN, GPIO_IN);
    gpio_pull_down(HCSR04_ECHO_PIN);
    gpio_set_irq_enabled_with_callback(
        HCSR04_ECHO_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &gpio_irq_handler
    );

    add_repeating_timer_ms(60, trig_on_callback, NULL, &_trig_timer);
}
