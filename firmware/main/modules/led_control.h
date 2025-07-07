#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "driver/gpio.h"

typedef enum {
    LED_STATE_OFF,
    LED_STATE_ON,
    LED_STATE_BLINK_SLOW,
    LED_STATE_BLINK_FAST,
    LED_STATE_BREATHING,
} led_state_t;

void led_control_init(void);
void led_control_set_state(led_state_t state);

#endif // LED_CONTROL_H
