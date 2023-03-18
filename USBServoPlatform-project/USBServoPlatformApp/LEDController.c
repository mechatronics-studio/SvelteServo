#include "pico/stdlib.h"
#include "stdio.h"
#include "LEDController.h"

bool LED_state = 0;
uint8_t LED_mode;

alarm_id_t led_transition_alarm;

int64_t (LED_blinking_1hz)(int32_t alarm_ID, void* empty){
    LED_state = !LED_state;
    gpio_put(PICO_DEFAULT_LED_PIN, !LED_state);
    return 1000000;
}

void LED_solid_on(){
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
}

void LED_solid_off(){
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

/**
 * This function initializes the LED pins in preparation for signaling.
*/
void initialize_LED(){
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    LED_mode = 0;
    LED_solid_off(); 
    #ifdef MASTER_DEBUG
                printf("%lld Core1: LED \n", time_us_64());
    #endif
}

/**
 * This function sets the LED Mode. 
 * Mode 1: LED Off
 * Mode 2: LED On
 * Mode 3: LED Blinking at 1 Hz.
*/
void set_LED_mode(uint8_t new_LED_mode, alarm_pool_t* alarm_pool_to_add_led_transition){
    if(new_LED_mode != LED_mode){
    switch(new_LED_mode){
            case 1:
                if(LED_mode == 3){
                alarm_pool_cancel_alarm(alarm_pool_to_add_led_transition, led_transition_alarm);
                }
                LED_solid_off();
                LED_mode = 1;
                #ifdef MASTER_DEBUG
                printf("%lld Core1: LED Set to Off\n", time_us_64());
                #endif
                break;
            
            case 2:
                if(LED_mode == 3){
                alarm_pool_cancel_alarm(alarm_pool_to_add_led_transition, led_transition_alarm);
                }
                LED_solid_on();
                LED_mode = 2;
                #ifdef MASTER_DEBUG
                printf("%lld Core1: LED Set to On\n", time_us_64());
                #endif
                break;
            case 3:{
                led_transition_alarm = alarm_pool_add_alarm_in_us(alarm_pool_to_add_led_transition,1000000,LED_blinking_1hz,NULL,false);
                LED_mode = 3;
                #ifdef MASTER_DEBUG
                printf("%lld Core1: LED Set to Blink at 1 Hz\n", time_us_64());
                #endif
                break;
            }

        }
    }

}