#include "pico/stdlib.h"
#include "pico/multicore.h"

void core1_main(){
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }
}

int main(){
    stdio_init_all();
    //Launch Second Core
    multicore_launch_core1(core1_main);
    while(1);
}