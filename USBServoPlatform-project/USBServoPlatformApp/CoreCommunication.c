#include "CoreCommunication.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "stdio.h"
#include "macro_header.h"

queue_t desired_state_variable_queue;
queue_t core1_command_queue;
queue_t core1_response_queue;
queue_t telemetry_output_queue;

uint8_t check_response_from_core1(uint8_t expected_executive_call, uint32_t expected_integer_response, float expected_float_response){
    if(queue_get_level(&core1_response_queue) > 0){
        core_command_response recieved_response;
        queue_remove_blocking(&core1_response_queue,&recieved_response);
        if(recieved_response.executive_call == expected_executive_call && recieved_response.supporting_input_integer == expected_integer_response, recieved_response.supporting_input_float == expected_float_response){
                return CORE1_RESPONSE_MATCHED_EXPECTED;
            }
            else{
                return CORE1_RESPONSE_DID_NOT_MATCH_EXPECTED;
            }
        }
    else{
        return CORE1_RESPONSE_NOT_YET_PROVIDED;
    }
}

uint8_t blocking_duration_check_for_response_from_core1(uint8_t expected_executive_call, uint32_t expected_integer_response, float expected_float_response, uint32_t microseconds_to_wait){
    uint32_t breakout_counter = 0;
    uint8_t core1_response_result = CORE1_RESPONSE_NOT_YET_PROVIDED;
    while(core1_response_result == CORE1_RESPONSE_NOT_YET_PROVIDED){
        core1_response_result = check_response_from_core1(expected_executive_call,expected_integer_response,expected_float_response);
        busy_wait_us(1);
        breakout_counter++;
        if(breakout_counter >= microseconds_to_wait){
            return CORE1_RESPONSE_TIMEOUT;
        }
    }
    return core1_response_result;
}

void send_command_to_core_1(uint8_t executive_call, int32_t supporting_input_integer, float supporting_input_float){
    core_command_response core_1_command = {.executive_call = executive_call, .supporting_input_integer = supporting_input_integer, .supporting_input_float = supporting_input_float};
    queue_add_blocking(&core1_command_queue,&core_1_command);
}

bool send_command_to_core1_and_wait_up_to_period_for_response(uint8_t executive_call, uint32_t supporting_input_integer, float supporting_input_float, uint32_t microseconds_to_wait){
    send_command_to_core_1(executive_call, supporting_input_integer, supporting_input_float);
    uint8_t core1_response = blocking_duration_check_for_response_from_core1(executive_call,supporting_input_integer,supporting_input_float,microseconds_to_wait);
    switch(core1_response){
        case CORE1_RESPONSE_TIMEOUT:
            printf("CE%d:CC, Core1 Failed to Provide Timely Response\n.", executive_call);
            return false;
            break;
        case CORE1_RESPONSE_DID_NOT_MATCH_EXPECTED:
            printf("CE%d:CC, Core1 Provided Incorrect Response\n",executive_call);
            return false;
            break;
        case CORE1_RESPONSE_MATCHED_EXPECTED:
            return true;
    }
}

void send_response_to_core_0(uint8_t executive_call, int32_t supporting_input_integer, float supporting_input_float){
    core_command_response core_1_response = {.executive_call = executive_call, .supporting_input_integer = supporting_input_integer, .supporting_input_float = supporting_input_float};
    queue_add_blocking(&core1_response_queue,&core_1_response);
}

/**
 * This function initializes the core communication module, which initalizes the core communication queues and sends a message to core0 that core1 is ready for commands.
 */
void initialize_core_communication(){
    queue_init(&desired_state_variable_queue, sizeof(int32_t), DESIRED_STATE_VARIABLE_QUEUE_SIZE);
    queue_init(&core1_command_queue, sizeof(core_command_response), 10);
    queue_init(&core1_response_queue, sizeof(core_command_response), 10);
    queue_init(&telemetry_output_queue, sizeof(telemetry_data), 10);
    #ifdef MASTER_DEBUG
    printf("%lld Core1: Queues initialized.\n", time_us_64());
    #endif
    send_response_to_core_0(CORE1_READY_EXECUTIVE_CALL,CORE1_READY_SUPPORTING_INPUT,CORE1_READY_SUPPORTING_INPUT);
}
