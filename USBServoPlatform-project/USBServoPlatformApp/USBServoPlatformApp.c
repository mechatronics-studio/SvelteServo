#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "CoreCommunication.h"
#include "USBCommunicationProtocol.h"
#include "Core0Commands.h"
#include "Core1Commands.h"
#include <stdio.h>
#include "macro_header.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "MT6701Encoder.h"


void core1_main(){
    #ifdef MASTER_DEBUG
    printf("%lld Core1: Core Started.\n", time_us_64());
    #endif

    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);

    setup_core1_peripherals();

    initialize_core_communication();

    #ifdef MASTER_DEBUG
    printf("%lld Core1: Ready for Commands from Core0.\n", time_us_64());
    #endif

    while(1){
        get_command_from_core0();

        //const float conversion_factor = 3.3f / (1 << 12) * 9.333333;
        //uint16_t result = adc_read();
        //printf("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
        //printf("%ld\n",(int32_t)result);

        //print_current_absolute_position();
        //printf("ENCODER VALUE: %ld\n",get_MT6701_quadrature_count());
        //busy_wait_ms(1000);
    }

}

int main(){

    initialize_usb_communication();

    #ifdef MASTER_DEBUG
    printf("%lld Core0: Boot\n", time_us_64());
    #endif

    multicore_launch_core1(core1_main);

    uint8_t core0_ready_response_recieved = CORE1_RESPONSE_NOT_YET_PROVIDED;

    while(core0_ready_response_recieved != CORE1_RESPONSE_MATCHED_EXPECTED){
        sleep_ms(100);
        core0_ready_response_recieved = check_response_from_core1(CORE1_READY_EXECUTIVE_CALL,CORE1_READY_SUPPORTING_INPUT,CORE1_READY_SUPPORTING_INPUT);
        #ifdef MASTER_DEBUG
        if(core0_ready_response_recieved == CORE1_RESPONSE_NOT_YET_PROVIDED){
        printf("%lld Core0: Core1 has not yet responded that it is ready.\n", time_us_64());
        }
        else if(core0_ready_response_recieved == CORE1_RESPONSE_DID_NOT_MATCH_EXPECTED){
        printf("%lld Core0: Core1 responded with the incorrect ready signal.\n", time_us_64());   
        }
        #endif
    }

    #ifdef MASTER_DEBUG
    printf("%lld Core0: Core1 has signaled it is ready for commands from Core0\n", time_us_64());
    #endif

    setup_core0_command_chain();

    #ifdef MASTER_DEBUG
    printf("%lld Core0: Ready for USB Commands\n", time_us_64());
    #endif

    while(1){
        if(command_read_complete == false){
            pop_char_from_usb_to_command_buffer();
        }
        else{
            execute_current_command_in_command_buffer();
        }
        
    }
}