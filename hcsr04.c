// #include "hcsr04.h"

// struct repeating_timer trig_timer;

// void gpio_irq_handler(uint gpio, uint32_t events) {
//     if (gpio == ECHO_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
//         hcsr04.start_time = to_us_since_boot(get_absolute_time());
//     } else if (gpio == ECHO_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
//         hcsr04.end_time = to_us_since_boot(get_absolute_time());
//         uint64_t duration = hcsr04.end_time - hcsr04.start_time;
//         hcsr04.distance = (duration * 0.0343f) / 2.0f;
//         hcsr04.measure_done = true;
//     }
// }

// int64_t trig_off_callback(alarm_id_t id, void *user_data) {
//     gpio_put(TRIG_PIN, 0);
//     return 0;
// }

// bool trig_on_callback(struct repeating_timer *t) {
//     gpio_put(TRIG_PIN, 1);
//     add_alarm_in_us(10, trig_off_callback, NULL, false);
//     return true;
// }