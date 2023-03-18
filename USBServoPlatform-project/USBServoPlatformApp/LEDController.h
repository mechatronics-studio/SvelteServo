#ifndef LED_CONTROLLER
#define LED_CONTROLLER
#include "pico/stdlib.h"

void set_LED_mode(uint8_t new_LED_mode, alarm_pool_t* alarm_pool_to_add_led_transition);

void initialize_LED();

#endif