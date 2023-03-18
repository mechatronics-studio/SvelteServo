#ifndef CORE_COMMUNICATION
#define CORE_COMMUNICATION
#include "pico/stdlib.h"
#include "stdio.h"
#include "pico/util/queue.h"

#define CORE1_RESPONSE_DID_NOT_MATCH_EXPECTED 0
#define CORE1_RESPONSE_MATCHED_EXPECTED 1
#define CORE1_RESPONSE_NOT_YET_PROVIDED 2
#define CORE1_RESPONSE_TIMEOUT 3

#define CORE1_READY_EXECUTIVE_CALL 255
#define CORE1_READY_SUPPORTING_INPUT 1

#define STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US 1000

void initialize_core_communication();

void send_command_to_core_1(uint8_t executive_call, int32_t supporting_input_integer, float supporting_input_float);

bool send_command_to_core1_and_wait_up_to_period_for_response(uint8_t executive_call, uint32_t supporting_input_integer, float supporting_input_float, uint32_t microseconds_to_wait);

void send_response_to_core_0(uint8_t executive_call, int32_t supporting_input_integer, float supporting_input_float);

uint8_t check_response_from_core1(uint8_t expected_executive_call, uint32_t expected_integer_response, float expected_float_response);

void get_command_from_core0();

extern queue_t desired_state_variable_queue;
extern queue_t core1_command_queue;
extern queue_t core1_response_queue;
extern queue_t telemetry_output_queue;

typedef struct
{
    int32_t time_us;
    int32_t desired_state_variable_des;
    int32_t desired_state_variable_act;
    int32_t control_voltage_mv;
} telemetry_data;

typedef struct
{
    uint8_t executive_call;
    int32_t supporting_input_integer;
    float supporting_input_float;
} core_command_response;

#endif