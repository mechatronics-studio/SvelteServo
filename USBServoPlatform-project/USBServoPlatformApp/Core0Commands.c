#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "macro_header.h"
#include "RamVariablesCore0.h"
#include "Core0Commands.h"
#include "USBCommunicationProtocol.h"
#include "CoreCommunication.h"

char bottango_effector_identifier[10] = "NA";
float min_bottango_effector_signal = 0;
float max_bottango_effector_signal = 0;
float max_bottango_signal_rate_signals_per_seconds = 0;
float start_bottango_effector_signal = 0;

uint32_t command_buffer_parsing_index = 0;

bool effector_added = false;

char word_buffer[WORD_BUFFER_MAX_LENGTH]; ///<The memory buffer that stores each word after being recieved.
uint32_t word_buffer_string_termination_index = 0;

char numeric_chars[11] = {'0','1','2','3','4','5','6','7','8','9','\0'};
char numeric_chars_plus_period[12] = {'0','1','2','3','4','5','6','7','8','9','.','\0'};
char negative_char = '-';
char period_string[2] = {'.','\0'};

bool word_buffer_is_integer_number(){
    uint32_t start_index = 0;
    if(word_buffer[0] == negative_char){
        start_index = 1;
    }

    if(strspn(&word_buffer[start_index],numeric_chars) == word_buffer_string_termination_index-start_index){
        return true;
    }
    else{
        return false;
    }
}

bool word_buffer_is_float_number(){
    uint32_t start_index = 0;
    if(word_buffer[0] == negative_char){
        start_index = 1;
    }

    uint32_t length_of_string_that_contains_numeric_chars_plus_period = strspn(&word_buffer[start_index],numeric_chars_plus_period);

    if(length_of_string_that_contains_numeric_chars_plus_period == word_buffer_string_termination_index-start_index){
        uint32_t index_of_first_period = strcspn(word_buffer,period_string);
        if(index_of_first_period == 0 && word_buffer_string_termination_index == 1){
            return false;
        }
        
        if(index_of_first_period != word_buffer_string_termination_index){
            char* word_buffer_after_first_period = &word_buffer[index_of_first_period+1];
            length_of_string_that_contains_numeric_chars_plus_period = strspn(word_buffer_after_first_period, numeric_chars);
            if(length_of_string_that_contains_numeric_chars_plus_period != word_buffer_string_termination_index - (index_of_first_period + 1)){
                return false;
            }
        }

        return true;
    }
    else{
        return false;
    }
}

void setup_core0_command_chain(){
    initiate_RAM_variables_core0();

    if(DEFAULT_TELEMETRY_REPORTING_MODE != 0){
    alarm_pool_add_alarm_in_us(core0_alarm_pool,telemetry_reporting_interval_us,transmit_telemetry_data_via_usb,NULL,false);
    }

    if(DEFAULT_ENABLE_SIMULATED_SERVO == 1){
        enable_simulated_servo_core0 = 1;
        send_command_to_core_1(CALL_21_SET_GET_ENABLE_SIMULATED_SERVO_COMMAND_NUM,enable_simulated_servo_core0,0);

    }
    if(DEFAULT_SERVO_ENABLE ==1){
        servo_enable_core0 = 1;
        send_command_to_core_1(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,servo_enable_core0,0);
    }

    printf("%s\n",BOTTANGO_BOOT_RESPONSE);
}

/**
 * Clear the word buffer by assigning the first element to the end of string delimiter.
*/
void clear_word_buffer(){
    word_buffer[0] = '\0';
    word_buffer_string_termination_index = 0;
}

/**
 * Get the next word from the command buffer and place it into the word buffer.
 * @return True if the next word was obtained, false if the command buffer has already been fully parsed and there is no new word to obtain.
*/
bool put_next_command_word_in_word_buffer(){
    if(command_buffer_parsing_index >= usb_command_buffer_string_termination_index){
        return false;
    }
    else{
        clear_word_buffer();
        while(command_buffer_parsing_index < usb_command_buffer_string_termination_index && command_buffer[command_buffer_parsing_index] != ',' && word_buffer_string_termination_index < WORD_BUFFER_MAX_LENGTH){
            word_buffer[word_buffer_string_termination_index] = command_buffer[command_buffer_parsing_index];
            command_buffer_parsing_index++;
            word_buffer_string_termination_index++;
            word_buffer[word_buffer_string_termination_index] = '\0';
        }
        if(command_buffer_parsing_index < usb_command_buffer_string_termination_index && command_buffer[command_buffer_parsing_index] == ','){
            command_buffer_parsing_index++;
        }
        return true;
    }
}

void add_desired_state_variable_to_desired_state_variable_queue(){
    int32_t desired_state_variable_to_add;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        desired_state_variable_to_add = atoi(word_buffer); 
    }
    else{
        printf("E%ld:1, State Variable Argument Not an Integer\n", CALL_0_ADD_SINGLE_DESIRED_STATE_VARIABLE_COMMAND_NUM);
        return;
    }

    if(queue_get_level(&desired_state_variable_queue)==DESIRED_STATE_VARIABLE_QUEUE_SIZE){
        printf("E%ld:2, State Variable Queue Full\n", CALL_0_ADD_SINGLE_DESIRED_STATE_VARIABLE_COMMAND_NUM);
        return;
    }
    else{
        bool queue_add_success = queue_try_add(&desired_state_variable_queue,&desired_state_variable_to_add);
        if(queue_add_success){
            if(debug_flags[CALL_0_ADD_SINGLE_DESIRED_STATE_VARIABLE_COMMAND_NUM] == 1){
                printf("S%ld, %ld\n", CALL_0_ADD_SINGLE_DESIRED_STATE_VARIABLE_COMMAND_NUM, desired_state_variable_to_add);
            }
            return;
        }
        else{
            printf("E%ld:3, Queue Add Fail\n", CALL_0_ADD_SINGLE_DESIRED_STATE_VARIABLE_COMMAND_NUM);
            return;
        }
    }
}

void add_multiple_desired_state_variables_to_desired_state_variable_queue(){
    int32_t number_of_state_variables_to_add;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        number_of_state_variables_to_add = atoi(word_buffer); 
    }
    else{
        printf("E%ld:1, State Variable Qty Argument Not an Integer\n", CALL_1_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_COMMAND_NUM);
        return;
    }

    if(number_of_state_variables_to_add < 1){
        printf("E%ld:2, State Variable Qty is <1\n", CALL_1_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_COMMAND_NUM);
        return;
    }

    int32_t state_variables_to_add[number_of_state_variables_to_add];
    for(int32_t i = 0; i < number_of_state_variables_to_add; i++){
        if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
            state_variables_to_add[i] = atoi(word_buffer); 
        }
        else{
            printf("E%ld:3, State Variable Argument %ld Not an Integer.\n", CALL_1_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_COMMAND_NUM, i+1);
            return;
        }
    }

    if(queue_get_level(&desired_state_variable_queue)>DESIRED_STATE_VARIABLE_QUEUE_SIZE-number_of_state_variables_to_add){
        printf("E%ld:4, State Variable Queue Full\n", CALL_1_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_COMMAND_NUM);
        return;
    }
    else{
        if(debug_flags[CALL_1_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_COMMAND_NUM] != 0){
            printf("S%ld, %ld ", CALL_1_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_COMMAND_NUM, number_of_state_variables_to_add);
        }
        for(int32_t i=0; i < number_of_state_variables_to_add; i++){
            bool queue_add_success = queue_try_add(&desired_state_variable_queue,&state_variables_to_add[i]);
            if(queue_add_success){
                if(debug_flags[CALL_1_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_COMMAND_NUM] == 1){
                    printf("%ld ", state_variables_to_add[i]);
                }
            }
            else{
                printf("E%ld:5, Queue Add Fail on on State Variable Argument %ld\n", CALL_1_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_COMMAND_NUM, i);
                return;
            }
        }
    }
    return;
}

void add_desired_state_variable_in_axis_output_units_to_desired_state_variable_queue(){
    float desired_state_variable_to_add_in_axis_output_units;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        desired_state_variable_to_add_in_axis_output_units = atof(word_buffer); 
    }
    else{
        printf("E%ld:1, State Variable Argument Not a Float\n", CALL_2_ADD_SINGLE_DESIRED_STATE_VARIABLE_IN_AXIS_OUTPUT_UNITS_COMMAND_NUM);
        return;
    }

    int32_t desired_state_variable_to_add;
    if(enable_simulated_servo_core0 == 1){
        desired_state_variable_to_add = desired_state_variable_to_add_in_axis_output_units*simulated_gear_ratio_counts_per_axis_output_unit;
    }
    else{
        desired_state_variable_to_add = desired_state_variable_to_add_in_axis_output_units*physical_gear_ratio_counts_per_axis_output_unit;
    }

    if(queue_get_level(&desired_state_variable_queue)==DESIRED_STATE_VARIABLE_QUEUE_SIZE){
        printf("E%ld:2, State Variable Queue Full\n", CALL_2_ADD_SINGLE_DESIRED_STATE_VARIABLE_IN_AXIS_OUTPUT_UNITS_COMMAND_NUM);
        return;
    }
    else{
        bool queue_add_success = queue_try_add(&desired_state_variable_queue,&desired_state_variable_to_add);
        if(queue_add_success){
            if(debug_flags[CALL_0_ADD_SINGLE_DESIRED_STATE_VARIABLE_COMMAND_NUM] == 1){
                printf("S%ld, %ld\n", CALL_2_ADD_SINGLE_DESIRED_STATE_VARIABLE_IN_AXIS_OUTPUT_UNITS_COMMAND_NUM, desired_state_variable_to_add);
            }
            return;
        }
        else{
            printf("E%ld:3, Queue Add Fail\n", CALL_2_ADD_SINGLE_DESIRED_STATE_VARIABLE_IN_AXIS_OUTPUT_UNITS_COMMAND_NUM);
            return;
        }
    }
}

void add_multiple_integers_in_axis_output_units_to_desired_state_variable_queue(){
    int32_t number_of_state_variables_to_add;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        number_of_state_variables_to_add = atoi(word_buffer); 
    }
    else{
        printf("E%ld:1, State Variable Qty Argument Not an Integer\n", CALL_3_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_IN_AXIS_OUTPUT_UNITS_COMMAND_NUM);
        return;
    }

    if(number_of_state_variables_to_add < 1){
        printf("E%ld:2, State Variable Qty is <1\n", CALL_3_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_IN_AXIS_OUTPUT_UNITS_COMMAND_NUM);
        return;
    }

    float state_variables_to_add_in_axis_output_units[number_of_state_variables_to_add];
    for(int32_t i = 0; i < number_of_state_variables_to_add; i++){
        if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
            state_variables_to_add_in_axis_output_units[i] = atof(word_buffer); 
        }
        else{
            printf("E%ld:3, State Variable Argument %ld Not a Float\n", CALL_3_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_IN_AXIS_OUTPUT_UNITS_COMMAND_NUM, i+1);
            return;
        }
    }

    int32_t state_variables_to_add[number_of_state_variables_to_add];
    for(int32_t i = 0; i < number_of_state_variables_to_add; i++){
        if(enable_simulated_servo_core0 == 1){
            state_variables_to_add[i] = state_variables_to_add_in_axis_output_units[i]*simulated_gear_ratio_counts_per_axis_output_unit;
        }
        else{
            state_variables_to_add[i] = state_variables_to_add_in_axis_output_units[i]*physical_gear_ratio_counts_per_axis_output_unit;
        }
    }

    if(queue_get_level(&desired_state_variable_queue)>DESIRED_STATE_VARIABLE_QUEUE_SIZE-number_of_state_variables_to_add){
        printf("E%ld:4, State Variable Queue Full\n", CALL_3_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_IN_AXIS_OUTPUT_UNITS_COMMAND_NUM);
        return;
    }
    else{
        if(debug_flags[CALL_3_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_IN_AXIS_OUTPUT_UNITS_COMMAND_NUM] != 0){
            printf("S%ld, %ld ", CALL_3_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_IN_AXIS_OUTPUT_UNITS_COMMAND_NUM, number_of_state_variables_to_add);
        }
        for(int32_t i=0; i < number_of_state_variables_to_add; i++){
            bool queue_add_success = queue_try_add(&desired_state_variable_queue,&state_variables_to_add[i]);
            if(queue_add_success){
                if(debug_flags[CALL_3_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_IN_AXIS_OUTPUT_UNITS_COMMAND_NUM] == 1){
                    printf("%ld ", state_variables_to_add[i]);
                }
            }
            else{
                printf("E%ld:5, Queue Add Fail on on State Variable Argument %ld\n", CALL_3_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_IN_AXIS_OUTPUT_UNITS_COMMAND_NUM, i);
                return;
            }
        }
    }
    return;
}

void set_get_servo_enable(){
    int32_t servo_enable;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        servo_enable = atoi(word_buffer); 
    }
    else{
        printf("E%ld:1, State Variable Argument Not an Integer\n", CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM);
        return;
    }

    if(servo_enable != 1 && servo_enable != -1){
        servo_enable = 0;
    }

    if(servo_enable == -1){
        printf("SERVO_ENABLE %ld\n", servo_enable_core0);
    }
    else{
        if(servo_enable != servo_enable_core0){
            servo_enable_core0 = servo_enable;
            send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,servo_enable_core0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            if(debug_flags[CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM] == 1){
                printf("SERVO ENABLE SET TO %u\n", servo_enable_core0);
            }
        }
    }
    return;
}

void set_get_servo_loop_interval(){
    uint32_t servo_loop_interval_dms;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        servo_loop_interval_dms = atoi(word_buffer);
    }
    else{
        printf("E%ld:1, Servo Loop Interval dms Argument Not an Integer\n", CALL_7_SET_GET_SERVO_LOOP_INTERVAL_COMMAND_NUM);
        return; 
    }

    if(servo_loop_interval_dms < 1 && servo_loop_interval_dms != -1){
        servo_loop_interval_dms = 1;
    }

    if(servo_loop_interval_dms == -1){
        printf("SERVO_UPDATE_INTERVAL_US %ld\n", servo_update_interval_us_core0);
    }
    else{
        int32_t servo_loop_interval_us = servo_loop_interval_dms*100;
        if(servo_update_interval_us_core0 != servo_loop_interval_us){
            servo_update_interval_us_core0 = servo_loop_interval_us;
            telemetry_reporting_interval_us = telemetry_reporting_interval_multiplier_core0*servo_update_interval_us_core0;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,servo_update_interval_us_core0/100,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_7_SET_GET_SERVO_LOOP_INTERVAL_COMMAND_NUM] == 1){
        printf("SERVO_LOOP_INTERVAL_US SET TO %ld\n", servo_update_interval_us_core0);
    }
}

void set_get_desired_state_variable_update_interval_multiplier(){
    uint32_t desired_state_variable_update_interval_multiplier;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        desired_state_variable_update_interval_multiplier = atoi(word_buffer);
    }
    else{
        printf("E%ld:1, Desired State Variable Update Interval Multiplier Argument Not an Integer\n", CALL_8_SET_GET_DESIRED_STATE_VARIABLE_UPDATE_INTERVAL_MULTIPLIER_COMMAND_NUM);
        return; 
    }

    if(desired_state_variable_update_interval_multiplier < 1 && desired_state_variable_update_interval_multiplier != -1){
        desired_state_variable_update_interval_multiplier = 1;
    }

    if(desired_state_variable_update_interval_multiplier == -1){
        printf("DESIRED_STATE_UPDATE_INTERVAL_MULTIPLIER %ld\n", desired_state_update_interval_multiplier_core0);
    }
    else{
        if(desired_state_update_interval_multiplier_core0 != desired_state_variable_update_interval_multiplier){
            desired_state_update_interval_multiplier_core0 = desired_state_variable_update_interval_multiplier;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_8_SET_GET_DESIRED_STATE_VARIABLE_UPDATE_INTERVAL_MULTIPLIER_COMMAND_NUM,desired_state_update_interval_multiplier_core0,0,1000);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_8_SET_GET_DESIRED_STATE_VARIABLE_UPDATE_INTERVAL_MULTIPLIER_COMMAND_NUM] == 1){
        printf("DESIRED_STATE_UPDATE_INTERVAL_MULTIPLIER SET TO %ld\n", desired_state_update_interval_multiplier_core0);
    }
}

void set_get_control_mode(){
    uint32_t control_mode;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        control_mode = atoi(word_buffer);
    }
    else{
        printf("E%ld:1, Control Mode Argument Not an Integer\n", CALL_9_SET_GET_CONTROL_MODE_COMMAND_NUM);
        return; 
    }


    if(control_mode != 1 && control_mode != 2 && control_mode != -1){
        control_mode = 1;
    }

    if(control_mode == -1){
        printf("CONTROL_MODE %ld\n", control_mode_core0);
    }
    else{
        if(control_mode_core0 != control_mode){
            control_mode_core0 = control_mode;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_9_SET_GET_CONTROL_MODE_COMMAND_NUM,control_mode_core0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

        }
    }
    if(debug_flags[CALL_9_SET_GET_CONTROL_MODE_COMMAND_NUM] == 1){
        printf("CONTROL_MODE SET TO %ld\n", control_mode_core0);
    }
}


void set_get_proportional_gain(){
    uint32_t proportional_gain;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        proportional_gain = atoi(word_buffer);
    }
    else{
        printf("E%ld:1, Proportional Gain Argument Not an Integer\n", CALL_10_SET_GET_PROPORTIONAL_GAIN_COMMAND_NUM);
        return; 
    }

    if(proportional_gain < 0 && proportional_gain != -1){
        proportional_gain = 0;
    }

    if(proportional_gain == -1){
        printf("PROPORTIONAL_GAIN %ld\n", proportional_gain_core0);
    }
    else{
        if(proportional_gain_core0 != proportional_gain){
            proportional_gain_core0 = proportional_gain;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_10_SET_GET_PROPORTIONAL_GAIN_COMMAND_NUM,proportional_gain_core0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_10_SET_GET_PROPORTIONAL_GAIN_COMMAND_NUM] == 1){
        printf("PROPORTIONAL_GAIN SET TO %ld\n", proportional_gain_core0);
    }
}

void set_get_derivative_gain(){
    uint32_t derivative_gain;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        derivative_gain = atoi(word_buffer);
    }
    else{
        printf("E%ld:1, Derivative Gain Argument Not an Integer\n", CALL_11_SET_GET_DERIVATIVE_GAIN_COMMAND_NUM);
        return; 
    }


    if(derivative_gain < 0 && derivative_gain != -1){
        derivative_gain = 0;
    }

    if(derivative_gain == -1){
        printf("DERIVATIVE_GAIN %ld\n", derivative_gain_core0);
    }
    else{
        if(derivative_gain_core0 != derivative_gain){
            derivative_gain_core0 = derivative_gain;


            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_11_SET_GET_DERIVATIVE_GAIN_COMMAND_NUM,derivative_gain_core0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_11_SET_GET_DERIVATIVE_GAIN_COMMAND_NUM] == 1){
        printf("DERIVATIVE_GAIN SET TO %ld\n", derivative_gain_core0);
    }
}

void set_get_integral_gain(){
    uint32_t integral_gain;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        integral_gain = atoi(word_buffer);
    }
    else{
        printf("E%ld:1, Integral Gain Argument Not an Integer\n", CALL_12_SET_GET_INTEGRAL_GAIN_COMMAND_NUM);
        return; 
    }


    if(integral_gain < 0 && integral_gain != -1){
        integral_gain = 0;
    }

    if(integral_gain == -1){
        printf("INTEGRAL_GAIN %ld\n", integral_gain_core0);
    }
    else{
        if(integral_gain_core0 != integral_gain){
            integral_gain_core0 = integral_gain;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_12_SET_GET_INTEGRAL_GAIN_COMMAND_NUM,integral_gain_core0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_12_SET_GET_INTEGRAL_GAIN_COMMAND_NUM] == 1){
        printf("INTEGRAL_GAIN SET TO %ld\n", integral_gain_core0);
    }
}

void set_get_enable_simulated_servo(){
    uint8_t enable_simulated_servo;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        enable_simulated_servo = atoi(word_buffer);
    }
    else{
        printf("E%ld:1, Enable Simulated Servo Argument Not an Integer\n", CALL_21_SET_GET_ENABLE_SIMULATED_SERVO_COMMAND_NUM);
        return; 
    }

    if(enable_simulated_servo != 1 && enable_simulated_servo != -1){
        enable_simulated_servo = 0;
    }
    if(enable_simulated_servo == -1){
        printf("ENABLE_SIMULATED_SERVO %ld\n", enable_simulated_servo_core0);
    }
    else{
        if(enable_simulated_servo_core0 != enable_simulated_servo){
            enable_simulated_servo_core0 = enable_simulated_servo;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_21_SET_GET_ENABLE_SIMULATED_SERVO_COMMAND_NUM,enable_simulated_servo_core0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_21_SET_GET_ENABLE_SIMULATED_SERVO_COMMAND_NUM] == 1){
        printf("ENABLE_SIMULATED_SERVO SET TO %ld\n", enable_simulated_servo_core0);
    }
}

void set_get_simulated_servo_loop_interval(){
    uint32_t simulated_servo_update_interval_us;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        simulated_servo_update_interval_us = atoi(word_buffer)*100;
    }
    else{
        printf("E%ld:1, Simulated Servo Loop Update Interval dms Argument Not an Integer\n", CALL_22_SET_GET_SIMULATED_SERVO_UPDATE_INTERVAL_COMMAND_NUM);
        return; 
    }

    if(simulated_servo_update_interval_us <0 && simulated_servo_update_interval_us != -1){
        simulated_servo_update_interval_us = 0;
    }
    if(simulated_servo_update_interval_us == -1){
        printf("SIMULATED_SERVO_UPDATE_INTERVAL_US %ld\n", simulated_servo_update_interval_us_core0);
    }
    else{
        if(simulated_servo_update_interval_us_core0 != simulated_servo_update_interval_us){
            simulated_servo_update_interval_us_core0 = simulated_servo_update_interval_us;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_22_SET_GET_SIMULATED_SERVO_UPDATE_INTERVAL_COMMAND_NUM,simulated_servo_update_interval_us_core0/100,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_22_SET_GET_SIMULATED_SERVO_UPDATE_INTERVAL_COMMAND_NUM] == 1){
        printf("SIMULATED_SERVO_UPDATE_INTERVAL_US SET TO %ld\n", enable_simulated_servo_core0);
    }
}

void set_get_simulated_servo_voltage(){
    uint32_t simulated_servo_voltage_mv;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        simulated_servo_voltage_mv = atoi(word_buffer);
    }
    else{
        printf("E%ld:1, Simulated Servo Voltage mV Argument Not an Integer\n", CALL_23_SET_GET_SIMULATED_SERVO_VOLTAGE_COMMAND_NUM);
        return; 
    }

    if(simulated_servo_voltage_mv <0 && simulated_servo_voltage_mv != -1){
        simulated_servo_voltage_mv = 0;
    }

    if(simulated_servo_voltage_mv == -1){
        printf("SIMULATED_SERVO_VOLTAGE_MV %ld\n", simulated_servo_voltage_mv_core0);
    }
    else{
        if(simulated_servo_voltage_mv_core0 != simulated_servo_voltage_mv){
            simulated_servo_voltage_mv_core0 = simulated_servo_voltage_mv;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_23_SET_GET_SIMULATED_SERVO_VOLTAGE_COMMAND_NUM,simulated_servo_voltage_mv_core0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_23_SET_GET_SIMULATED_SERVO_VOLTAGE_COMMAND_NUM] == 1){
        printf("SIMULATED_SERVO_VOLTAGE_MV SET TO %ld\n", simulated_servo_voltage_mv_core0);
    }
}

void set_get_simulated_servo_torque_constant(){
    float simulated_servo_torque_constant_Nm_per_amp;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        simulated_servo_torque_constant_Nm_per_amp = atof(word_buffer);
    }
    else{
        printf("E%ld:1, Simulated Servo Torque Constant Nm per Amp Argument Not a Floating Point Number\n", CALL_24_SET_GET_SIMULATED_SERVO_TORQUE_CONSTANT_COMMAND_NUM);
        return; 
    }

    if(simulated_servo_torque_constant_Nm_per_amp <0 && simulated_servo_torque_constant_Nm_per_amp != -1){
        simulated_servo_torque_constant_Nm_per_amp = 0;
    }
    if(simulated_servo_torque_constant_Nm_per_amp == -1){
        printf("SIMULATED_SERVO_TORQUE_CONSTANT_NM_PER_AMP %0.9f\n", simulated_servo_torque_constant_Nm_per_amp_core0);
    }
    else{
        if(simulated_servo_torque_constant_Nm_per_amp_core0 != simulated_servo_torque_constant_Nm_per_amp){
            simulated_servo_torque_constant_Nm_per_amp_core0 = simulated_servo_torque_constant_Nm_per_amp;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_24_SET_GET_SIMULATED_SERVO_TORQUE_CONSTANT_COMMAND_NUM,0,simulated_servo_torque_constant_Nm_per_amp_core0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_24_SET_GET_SIMULATED_SERVO_TORQUE_CONSTANT_COMMAND_NUM] == 1){
        printf("SIMULATED_SERVO_TORQUE_CONSTANT_NM_PER_AMP SET TO %0.9f\n", simulated_servo_torque_constant_Nm_per_amp_core0);
    }
}

void set_get_simulated_servo_winding_resistance(){
    float simulated_servo_winding_resistance_ohms;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        simulated_servo_winding_resistance_ohms = atof(word_buffer);
    }
    else{
        printf("E%ld:1, Simulated Servo Winding Resistance Ohms Argument Not a Floating Point Number\n", CALL_25_SET_GET_SIMULATED_SERVO_WINDING_RESISTANCE_COMMAND_NUM);
        return; 
    }

    if(simulated_servo_winding_resistance_ohms < MINIMUM_SIMULATED_SERVO_WINDING_RESISTANCE_OHMS && simulated_servo_winding_resistance_ohms != -1){
        simulated_servo_winding_resistance_ohms = MINIMUM_SIMULATED_SERVO_WINDING_RESISTANCE_OHMS;
    }
    if(simulated_servo_winding_resistance_ohms == -1){
        printf("SIMULATED_SERVO_WINDING_RESISTANCE_OHMS %0.9f\n", simulated_servo_winding_resistance_ohms_core0);
    }
    else{
        if(simulated_servo_winding_resistance_ohms_core0 != simulated_servo_winding_resistance_ohms){
            simulated_servo_winding_resistance_ohms_core0 = simulated_servo_winding_resistance_ohms;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_25_SET_GET_SIMULATED_SERVO_WINDING_RESISTANCE_COMMAND_NUM,0,simulated_servo_winding_resistance_ohms_core0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_25_SET_GET_SIMULATED_SERVO_WINDING_RESISTANCE_COMMAND_NUM] == 1){
        printf("SIMULATED_SERVO_WINDING_RESISTANCE_OHMS SET TO %0.9f\n", simulated_servo_winding_resistance_ohms_core0);
    }
}

void set_get_simulated_servo_counts_per_revolution(){
    uint32_t simulated_servo_counts_per_revolution;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        simulated_servo_counts_per_revolution = atol(word_buffer);
    }
    else{
        printf("E%ld:1, Simulated Servo Counts Per Revolution Argument Not an Integer\n", CALL_26_SET_GET_SIMULATED_SERVO_COUNTS_PER_REVOLUTION_COMMAND_NUM);
        return; 
    }

    if(simulated_servo_counts_per_revolution <1 && simulated_servo_counts_per_revolution != -1){
        simulated_servo_counts_per_revolution = 1;
    }
    if(simulated_servo_counts_per_revolution == -1){
        printf("SIMULATED_SERVO_COUNTS_PER_REVOLUTION %ld\n", simulated_servo_counts_per_revolution_core0);
    }
    else{
        if(simulated_servo_counts_per_revolution_core0 != simulated_servo_counts_per_revolution){
            simulated_servo_counts_per_revolution_core0 = simulated_servo_counts_per_revolution;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_26_SET_GET_SIMULATED_SERVO_COUNTS_PER_REVOLUTION_COMMAND_NUM,simulated_servo_counts_per_revolution_core0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_26_SET_GET_SIMULATED_SERVO_COUNTS_PER_REVOLUTION_COMMAND_NUM] == 1){
        printf("SIMULATED_SERVO_COUNTS_PER_REVOLUTION SET TO %ld\n", simulated_servo_counts_per_revolution_core0);
    }
}

void set_get_simulated_servo_inertia(){
    float simulated_servo_inertia_kg_m2;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        simulated_servo_inertia_kg_m2 = atof(word_buffer);
    }
    else{
        printf("E%ld:1, Simulated Servo Inertia Kg*m^2 Argument Is Not a Floating Point Number\n", CALL_27_SET_GET_SIMULATED_SERVO_INERTIA_COMMAND_NUM);
        return; 
    }

    if(simulated_servo_inertia_kg_m2 < MINIMUM_SIMULATED_SERVO_INERTIA_KG_M2 && simulated_servo_inertia_kg_m2 != -1){
        simulated_servo_inertia_kg_m2 = MINIMUM_SIMULATED_SERVO_INERTIA_KG_M2;
    }
    if(simulated_servo_inertia_kg_m2 == -1){
        printf("SIMULATED_SERVO_INERTIA_KG_M2 %0.9f\n", simulated_servo_inertia_kg_m2_core0);
    }
    else{
        if(simulated_servo_inertia_kg_m2_core0 != simulated_servo_inertia_kg_m2){
            simulated_servo_inertia_kg_m2_core0 = simulated_servo_inertia_kg_m2;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_27_SET_GET_SIMULATED_SERVO_INERTIA_COMMAND_NUM,0,simulated_servo_inertia_kg_m2_core0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_27_SET_GET_SIMULATED_SERVO_INERTIA_COMMAND_NUM] == 1){
        printf("SIMULATED_SERVO_INERTIA_KG_M2 SET TO %0.9f\n", simulated_servo_winding_resistance_ohms_core0);
    }
}

void set_get_simulated_servo_damping_drag_coefficient(){
    float simulated_servo_damping_drag_coefficient_Nm_sec_per_rad;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        simulated_servo_damping_drag_coefficient_Nm_sec_per_rad = atof(word_buffer);
    }
    else{
        printf("E%ld:1, Simulated Servo Damping Drag Coefficient Nm*sec/rad Argument Is Not a Floating Point Number\n", CALL_28_SET_GET_SIMULATED_SERVO_DRAG_COEFFICIENT_COMMAND_NUM);
        return; 
    }

    if(simulated_servo_damping_drag_coefficient_Nm_sec_per_rad <0 && simulated_servo_damping_drag_coefficient_Nm_sec_per_rad != -1){
        simulated_servo_damping_drag_coefficient_Nm_sec_per_rad = 0;
    }
    if(simulated_servo_damping_drag_coefficient_Nm_sec_per_rad == -1){
        printf("SIMULATED_SERVO_DAMPING_DRAG_COEFFICIENT_NM_SEC_PER_RAD %0.9f\n", simulated_servo_damping_drag_coefficient_Nm_sec_per_rad_core0);
    }
    else{
        if(simulated_servo_damping_drag_coefficient_Nm_sec_per_rad_core0 != simulated_servo_damping_drag_coefficient_Nm_sec_per_rad){
            simulated_servo_damping_drag_coefficient_Nm_sec_per_rad_core0 = simulated_servo_damping_drag_coefficient_Nm_sec_per_rad;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_28_SET_GET_SIMULATED_SERVO_DRAG_COEFFICIENT_COMMAND_NUM,0,simulated_servo_damping_drag_coefficient_Nm_sec_per_rad_core0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_28_SET_GET_SIMULATED_SERVO_DRAG_COEFFICIENT_COMMAND_NUM] == 1){
        printf("SIMULATED_SERVO_DAMPING_DRAG_COEFFICIENT_NM_SEC_PER_RAD SET TO %0.9f\n", simulated_servo_damping_drag_coefficient_Nm_sec_per_rad_core0);
    }
}

void set_get_simulated_servo_kinetic_friction_torque(){
    float simulated_servo_kinetic_friction_torque_Nm;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        simulated_servo_kinetic_friction_torque_Nm = atof(word_buffer);
    }
    else{
        printf("E%ld:1, Simulated Servo Kinetic Friction Torque Nm Argument Is Not a Floating Point Number\n", CALL_29_SET_GET_SIMULATED_SERVO_KINETIC_FRICTION_TORQUE_COMMAND_NUM);
        return; 
    }

    if(simulated_servo_kinetic_friction_torque_Nm <0 && simulated_servo_kinetic_friction_torque_Nm != -1){
        simulated_servo_kinetic_friction_torque_Nm = 0;
    }
    if(simulated_servo_kinetic_friction_torque_Nm == -1){
        printf("SIMULATED_SERVO_KINETIC_FRICTION_TORQUE_NM %0.9f\n", simulated_servo_kinetic_friction_torque_Nm_core0);
    }
    else{
        if(simulated_servo_kinetic_friction_torque_Nm_core0 != simulated_servo_kinetic_friction_torque_Nm){
            simulated_servo_kinetic_friction_torque_Nm_core0 = simulated_servo_kinetic_friction_torque_Nm;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_29_SET_GET_SIMULATED_SERVO_KINETIC_FRICTION_TORQUE_COMMAND_NUM,0,simulated_servo_kinetic_friction_torque_Nm_core0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_29_SET_GET_SIMULATED_SERVO_KINETIC_FRICTION_TORQUE_COMMAND_NUM] == 1){
        printf("SIMULATED_SERVO_KINETIC_FRICTION_TORQUE_NM SET TO %0.9f\n", simulated_servo_kinetic_friction_torque_Nm_core0);
    }
}

void set_get_simulated_gear_ratio(){
    float new_simulated_gear_ratio;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        new_simulated_gear_ratio = atof(word_buffer);
    }
    else{
        printf("E%ld:1, Simulated Gear Ratio Argument Is Not a Floating Point Number\n", CALL_30_SET_GET_SIMULATED_SERVO_GEAR_RATIO_COMMAND_NUM);
        return; 
    }
 
    if(new_simulated_gear_ratio <1 && new_simulated_gear_ratio != -1){
        new_simulated_gear_ratio = 1;
    }
    if(new_simulated_gear_ratio == -1){
        printf("SIMULATED_GEAR_RATIO_COUNTS_PER_AXIS_OUTPUT_UNIT %0.9f\n", simulated_gear_ratio_counts_per_axis_output_unit);
    }
    else{
        if(simulated_gear_ratio_counts_per_axis_output_unit != new_simulated_gear_ratio){
            simulated_gear_ratio_counts_per_axis_output_unit = new_simulated_gear_ratio;
        }
    }
    if(debug_flags[CALL_30_SET_GET_SIMULATED_SERVO_GEAR_RATIO_COMMAND_NUM] == 1){
        printf("SIMULATED_GEAR_RATIO_COUNTS_PER_AXIS_OUTPUT_UNIT SET TO %0.9f\n", simulated_gear_ratio_counts_per_axis_output_unit);
    }
}

void set_get_physical_gear_ratio(){
    float new_physical_gear_ratio;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        new_physical_gear_ratio = atof(word_buffer);
    }
    else{
        printf("E%ld:1, Physical Gear Ratio Argument Is Not a Floating Point Number\n", CALL_31_SET_GET_PHYSICAL_GEAR_RATIO_COMMAND_NUM);
        return; 
    }

    if(new_physical_gear_ratio <1 && new_physical_gear_ratio != -1){
        new_physical_gear_ratio = 1;
    }
    if(new_physical_gear_ratio == -1){
        printf("PHYSICAL_GEAR_RATIO_COUNTS_PER_AXIS_OUTPUT_UNIT %0.9f\n", physical_gear_ratio_counts_per_axis_output_unit);
    }
    else{
        if(physical_gear_ratio_counts_per_axis_output_unit != new_physical_gear_ratio){
            physical_gear_ratio_counts_per_axis_output_unit = new_physical_gear_ratio;
        }
    }
    if(debug_flags[CALL_31_SET_GET_PHYSICAL_GEAR_RATIO_COMMAND_NUM] == 1){
        printf("PHYSICAL_GEAR_RATIO_COUNTS_PER_AXIS_OUTPUT_UNIT SET TO %0.9f\n", physical_gear_ratio_counts_per_axis_output_unit);
    }
}

void set_get_static_motor_voltage(){
    uint32_t static_motor_power_supply_voltage_mv;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        static_motor_power_supply_voltage_mv = atol(word_buffer);
    }
    else{
        printf("E%ld:1, Static Motor Voltage mV Argument Not an Integer\n", CALL_33_SET_GET_STATIC_MOTOR_POWER_SUPPLY_VOLTAGE_COMMAND_NUM);
        return; 
    }
    if(static_motor_power_supply_voltage_mv == -1){
        printf("STATIC_MOTOR_POWER_SUPPLY_VOLTAGE_MV %ld\n", static_motor_power_supply_voltage_mv_core0);
    }
    else{
        if(static_motor_power_supply_voltage_mv_core0 != static_motor_power_supply_voltage_mv){
            static_motor_power_supply_voltage_mv_core0 = static_motor_power_supply_voltage_mv;

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }

            send_command_to_core1_and_wait_up_to_period_for_response(CALL_33_SET_GET_STATIC_MOTOR_POWER_SUPPLY_VOLTAGE_COMMAND_NUM,static_motor_power_supply_voltage_mv_core0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);

            if(servo_enable_core0 == 1){
                send_command_to_core1_and_wait_up_to_period_for_response(CALL_6_SET_GET_SERVO_ENABLE_COMMAND_NUM,1,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
            }
        }
    }
    if(debug_flags[CALL_33_SET_GET_STATIC_MOTOR_POWER_SUPPLY_VOLTAGE_COMMAND_NUM] == 1){
        printf("STATIC_MOTOR_POWER_SUPPLY_VOLTAGE_MV SET TO %ld\n", static_motor_power_supply_voltage_mv_core0);
    }
}

void clear_desired_state_variable_queue(){
    uint32_t size_of_queue = queue_get_level(&desired_state_variable_queue);
    uint32_t data_trashing_buffer;
    for(uint32_t i = 0; i < size_of_queue; i++){
        queue_try_remove(&desired_state_variable_queue,&data_trashing_buffer);
    }
}

void set_get_telemetry_reporting_mode(){
    uint32_t telemetry_reporting_mode;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        telemetry_reporting_mode = atol(word_buffer);
    }
    else{
        printf("E%ld:1, Telemetry Reporting Mode Argument Not an Integer\n", CALL_38_SET_GET_TELEMETRY_REPORTING_MODE_COMMAND_NUM);
        return; 
    }
    if(telemetry_reporting_mode != 1 && telemetry_reporting_mode != 2 && telemetry_reporting_mode != 3 &&  telemetry_reporting_mode != -1){
        telemetry_reporting_mode = 0;
    }
    if(telemetry_reporting_mode == -1){
        printf("TELEMETRY_REPORTING_MODE %ld\n", control_mode_core0);
    }
    else{
        if(telemetry_reporting_mode_core0 != telemetry_reporting_mode){
            telemetry_reporting_mode_core0 = telemetry_reporting_mode;
            send_command_to_core1_and_wait_up_to_period_for_response(CALL_38_SET_GET_TELEMETRY_REPORTING_MODE_COMMAND_NUM,telemetry_reporting_mode_core0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
        }
    }
    if(debug_flags[CALL_38_SET_GET_TELEMETRY_REPORTING_MODE_COMMAND_NUM] == 1){
        printf("TELEMETRY_REPORTING_MODE SET TO %ld\n", telemetry_reporting_mode_core0);
    }
}

void set_get_telemetry_reporting_interval_multiplier(){
    uint32_t telemetry_reporting_interval_multiplier;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        telemetry_reporting_interval_multiplier = atol(word_buffer);
    }
    else{
        printf("E%ld:1, Telemetry Reporting Interval Multiplier Argument Not an Integer\n", CALL_39_SET_GET_TELEMETRY_REPORTING_INTERVAL_MULTIPLIER_COMMAND_NUM);
        return; 
    }

    if(telemetry_reporting_interval_multiplier < 1 && telemetry_reporting_interval_multiplier != -1){
        telemetry_reporting_interval_multiplier = 1;
    }
    if(telemetry_reporting_interval_multiplier == -1){
        printf("TELEMETRY_REPORTING_INTERVAL_MULTIPLIER %ld\n", telemetry_reporting_interval_multiplier_core0);
    }
    else{
        if(telemetry_reporting_interval_multiplier_core0 != telemetry_reporting_interval_multiplier){
            telemetry_reporting_interval_multiplier_core0 = telemetry_reporting_interval_multiplier;
            send_command_to_core1_and_wait_up_to_period_for_response(CALL_39_SET_GET_TELEMETRY_REPORTING_INTERVAL_MULTIPLIER_COMMAND_NUM,telemetry_reporting_interval_multiplier_core0,0,STANDARD_CORE1_RESPONSE_WAITING_PERIOD_US);
        }
    }
    if(debug_flags[CALL_39_SET_GET_TELEMETRY_REPORTING_INTERVAL_MULTIPLIER_COMMAND_NUM] == 1){
        printf("TELEMETRY_REPORTING_INTERVAL_MULTIPLIER SET TO %ld\n", telemetry_reporting_interval_multiplier_core0);
    }
}

void set_get_debug_flag(){
    uint32_t executive_call_flag_number;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        executive_call_flag_number = atol(word_buffer);
    }
    else{
        printf("E%ld:1, Executive Call Argument Not an Integer\n", CALL_40_SET_GET_DEBUG_FLAG_COMMAND_NUM);
        return; 
    }


    uint32_t debug_flag_value;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        debug_flag_value = atol(word_buffer);
    }
    else{
        printf("E%ld:2, Debug Flag Argument Not an Integer\n", CALL_40_SET_GET_DEBUG_FLAG_COMMAND_NUM);
        return; 
    }

    if(executive_call_flag_number < -1 || executive_call_flag_number >= QTY_OF_NUMERICAL_COMMANDS){
        printf("E%ld:3, Executive Call Argument Not Valid\n", CALL_40_SET_GET_DEBUG_FLAG_COMMAND_NUM);
        return; 
    }

    if(debug_flag_value == -1){
        printf("EXEC_CALL %ld DEBUG_FLAG %ld\n", executive_call_flag_number, debug_flags[executive_call_flag_number]);
    }
    else{
        if(debug_flag_value < 0){
            debug_flag_value = 0;
        }
        debug_flags[executive_call_flag_number] = debug_flag_value; 
    }

    if(debug_flags[CALL_40_SET_GET_DEBUG_FLAG_COMMAND_NUM] == 1){
        printf("EXEC_CALL %ld DEBUG_FLAG %ld SET TO %ld\n", executive_call_flag_number, debug_flags[executive_call_flag_number]);
    }
}

void set_get_multiple_debug_flags(){
    uint32_t number_of_debug_flags_to_set;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        number_of_debug_flags_to_set = atol(word_buffer);
    }
    else{
        printf("E%ld:1, Number of Debug Flags to Set Argument Not an Integer\n", CALL_41_SET_MULTIPLE_DEBUG_FLAGS_COMMAND_NUM);
        return; 
    }

    for(uint32_t i = 0; i < number_of_debug_flags_to_set; i++){
        uint32_t executive_call_flag_number;
        if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
            executive_call_flag_number = atol(word_buffer);
        }
        else{
            printf("E%ld:2, Executive Call Argument Not an Integer\n", CALL_41_SET_MULTIPLE_DEBUG_FLAGS_COMMAND_NUM);
            return; 
        }

        uint32_t debug_flag_value;
        if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
            debug_flag_value = atol(word_buffer);
        }
        else{
            printf("E%ld:3, Debug Flag Argument Not an Integer\n", CALL_41_SET_MULTIPLE_DEBUG_FLAGS_COMMAND_NUM);
            return; 
        }


        if(executive_call_flag_number < -1 || executive_call_flag_number >= QTY_OF_NUMERICAL_COMMANDS){
            printf("E%ld:4, Executive Call Argument Not Valid\n", CALL_41_SET_MULTIPLE_DEBUG_FLAGS_COMMAND_NUM);
            return; 
        }
        if(debug_flag_value == -1){
            printf("EXEC_CALL %ld DEBUG_FLAG %ld\n", executive_call_flag_number, debug_flags[executive_call_flag_number]);
        }
        else{
            if(debug_flag_value < 0){
                debug_flag_value = 0;
            }
            debug_flags[executive_call_flag_number] = debug_flag_value; 
        }
        if(debug_flags[CALL_41_SET_MULTIPLE_DEBUG_FLAGS_COMMAND_NUM] == 1){
        printf("EXEC_CALL %ld DEBUG_FLAG %ld SET TO %ld\n", executive_call_flag_number, debug_flags[executive_call_flag_number]);
    }
    }
}


void execute_numerical_command(uint32_t executive_call_num){
    switch(executive_call_num){
        case CALL_0_ADD_SINGLE_DESIRED_STATE_VARIABLE_COMMAND_NUM:
            add_desired_state_variable_to_desired_state_variable_queue();
            break;
        
        case CALL_1_ADD_MULTIPLE_DESIRED_STATE_VARIABLES_COMMAND_NUM:
            add_multiple_desired_state_variables_to_desired_state_variable_queue();
            break; 

        case 2: //Add Single Desired State Variable in Axis Output Units
            add_desired_state_variable_in_axis_output_units_to_desired_state_variable_queue();
            break; 

        case 3: //Add Multiple Desired State Variables in Axis Output Units
            add_multiple_integers_in_axis_output_units_to_desired_state_variable_queue();
            break; 

        case 4: //Reserved
            printf("E%u EXECUTIVE CALL NOT VALID\n", 4);
            break;

        case 5: //Reserved
            printf("E%u EXECUTIVE CALL NOT VALID\n", 5);
            break; 

        case 6: //Set|Get Servo Enable
            set_get_servo_enable();
            break; 

        case 7: //Set|Get Servo Loop Interval
            set_get_servo_loop_interval();
            break; 

        case 8: //Set|Get Desired State Variable Update Interval Multiplier
            set_get_desired_state_variable_update_interval_multiplier();
            break;

        case 9: //Set|Get Control Mode
            set_get_control_mode();
            break; 

        case 10: //Set|Get Proportional Gain
            set_get_proportional_gain();
            break; 

        case 11: //Set|Get Derivative Gain
            set_get_derivative_gain();
            break; 

        case 12: //Set|Get Integral Gain
            set_get_integral_gain();
            break;

        case 13: //Reserved
            printf("E%u EXECUTIVE CALL NOT VALID\n", 13);
            break; 

        case 14: //Reserved
            printf("E%u EXECUTIVE CALL NOT VALID\n", 14);
            break; 

        case 15: //Reserved
            printf("E%u EXECUTIVE CALL NOT VALID\n", 15);
            break; 

        case 16: //Deadhead Home Servo Blocking TO BE IMPLEMENTED LATER
            printf("E%u EXECUTIVE CALL NOT YET SUPPORTED\n", 16);
            break;

        case 17: //Set|Get Deadhead Home Voltage Limit TO BE IMPLEMENTED LATER
            printf("E%u EXECUTIVE CALL NOT YET SUPPORTED\n", 17);
            break; 

        case 18: //Set|Get Deadhead Home Timeout TO BE IMPLEMENTED LATER
            printf("E%u EXECUTIVE CALL NOT YET SUPPORTED\n", 18);
            break; 

        case 19: //Set|Get Deadhead Home Velocity TO BE IMPLEMENTED LATER
            printf("E%u EXECUTIVE CALL NOT YET SUPPORTED\n", 19);
            break; 

        case 20: //Set|Get Deadhead Home Velocity in Axis Output Units TO BE IMPLEMENTED LATER
            printf("E%u EXECUTIVE CALL NOT YET SUPPORTED\n", 20);
            break;

        case 21: //Set|Get Enable Simulated Servo
            set_get_enable_simulated_servo();
            break; 

        case 22: //Set|Get Simulated Servo Update Interval
            set_get_simulated_servo_loop_interval();
            break; 

        case 23: //Set|Get Simulated Motor Voltage
            set_get_simulated_servo_voltage();
            break; 

        case 24: //Set|Get Simulated Servo Torque Constant
            set_get_simulated_servo_torque_constant();
            break;

        case 25: //Set|Get Simulated Servo Winding Resistance
            set_get_simulated_servo_winding_resistance();
            break; 

        case 26: //Set|Get Simulated Servo Counts Per Revolution
            set_get_simulated_servo_counts_per_revolution();
            break; 

        case 27: //Set|Get Simulated Servo Inertia
            set_get_simulated_servo_inertia();
            break; 

        case 28: //Set|Get Simulated Servo Drag Coefficient
            set_get_simulated_servo_damping_drag_coefficient();
            break;

        case 29: //Set|Get Simulated Servo Kinetic Friction Torque 
            set_get_simulated_servo_kinetic_friction_torque();
            break; 

        case 30: //Set|Get Simulated Gear Ratio
            set_get_simulated_gear_ratio();
            break; 

        case 31: //Set|Get Physical Gear Ratio
            set_get_physical_gear_ratio();
            break; 

        case 32: //Measure Motor Power Supply
            printf("E%u EXECUTIVE CALL NOT YET SUPPORTED\n", 20);
            break; 

        case 33: //Set|Get Static Motor Power Supply Voltage
            set_get_static_motor_voltage();
            break; 

        case 34: //Set|Get Device Name
            printf("E%u EXECUTIVE CALL NOT YET SUPPORTED\n", 20);
            break;

        case 35: //Set|Get Position in Counts
            printf("E%u EXECUTIVE CALL NOT YET SUPPORTED\n", 20);
            break; 

        case 36: //Set|Get Position in Axis Output Units
            printf("E%u EXECUTIVE CALL NOT YET SUPPORTED\n", 20);
            break; 

        case 37: //Clear Disired State Variable Queue
            clear_desired_state_variable_queue();
            break; 

        case 38: //Set|Get Telemetry Reporting Mode
            set_get_telemetry_reporting_mode();
            break;

        case 39: //Set|Get Telemetry Reporting Interval Multiplier
            set_get_telemetry_reporting_interval_multiplier();
            break; 

        case 40: //Set|Get Debug Flag
            set_get_debug_flag();
            break; 

        case 41: //Set Multiple Debug Flags
            set_get_multiple_debug_flags();
            break; 

        case 42: //Set|Get Error Voltage Limit
            printf("E%u EXECUTIVE CALL NOT YET SUPPORTED\n", 42);
            break;

        case 43: //Set|Get Error Position Limit
            printf("E%u EXECUTIVE CALL NOT YET SUPPORTED\n", 43);
            break; 

        case 44: //Set|Get Error Position Limit in Axis Output Units
            printf("E%u EXECUTIVE CALL NOT YET SUPPORTED\n", 44);
            break; 

        case 45: //Set|Get Error Velocity Limit
            printf("E%u EXECUTIVE CALL NOT YET SUPPORTED\n", 45);
            break; 

        case 46: //Set|Get Error Velocity Limit in Axis Output Units
            printf("E%u EXECUTIVE CALL NOT YET SUPPORTED\n", 46);
            break; 

        default:
            break;
    }
}

void execute_string_command(char* word_buffer){}


void bottango_handshake(){
    int32_t recieved_code;
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        recieved_code = atoi(word_buffer);
        bottango_handshake(recieved_code);
    }
    else{
        printf("E_%s, Recieved Code Argument Not an Integer\n",BOTTANGO_HANDSHAKE_REQUEST_COMMAND);
        return;
    }
    printf("%s,%s,%i,%i\n",BOTTANGO_HANDSHAKE_RESPONSE,BOTTANGO_DRIVER_VERSION,recieved_code,1);
    bottango_session_connected = true;
}

void bottango_time_sync(){
    if(put_next_command_word_in_word_buffer() && word_buffer_is_integer_number()){
        host_time_us_at_last_sync = atoi(word_buffer)*1000;
        device_time_minus_host_time_us = time_us_64() - host_time_us_at_last_sync;
    }
    else{
        printf("E_%s, Recieved Time Sync Argument Not an Integer\n",BOTTANGO_TIME_SYNC_COMMAND);
    }
}

void bottango_add_effector(){
    char identifier_temp[10];
    float min_signal_temp;
    float max_signal_temp;
    float max_signal_rate_signals_per_seconds_temp;
    float start_signal_temp;

    if(put_next_command_word_in_word_buffer()){
        strcpy(identifier_temp,word_buffer);
    }
    else{
        printf("E_%s, Recieved Identifier Command Not a String\n",BOTTANGO_REGISTER_CUSTOM_MOTOR_COMMAND);
        return;
    }

    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        min_signal_temp = atoi(word_buffer);
    }
    else{
        printf("E_%s, Recieved Minimum Signal Value Not a Number\n",BOTTANGO_REGISTER_CUSTOM_MOTOR_COMMAND);
        return;
    }

    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        max_signal_temp = atoi(word_buffer);
    }
    else{
        printf("E_%s, Recieved Maximum Signal Value Not a Number\n",BOTTANGO_REGISTER_CUSTOM_MOTOR_COMMAND);
        return;
    }

    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        max_signal_rate_signals_per_seconds_temp = atoi(word_buffer);
        if(max_signal_rate_signals_per_seconds_temp < 0){
            max_signal_rate_signals_per_seconds_temp = -1*max_signal_rate_signals_per_seconds_temp;
        }
    }
    else{
        printf("E_%s, Recieved Maximum Signal Rate Value Not a Number\n",BOTTANGO_REGISTER_CUSTOM_MOTOR_COMMAND);
        return;
    }

    if(put_next_command_word_in_word_buffer() && word_buffer_is_float_number()){
        start_signal_temp = atoi(word_buffer);
    }
    else{
        printf("E_%s, Recieved Start Signal Value Not a Number\n",BOTTANGO_REGISTER_CUSTOM_MOTOR_COMMAND);
        return;
    }

    strcpy(bottango_effector_identifier,identifier_temp);
    min_bottango_effector_signal = min_signal_temp;
    max_bottango_effector_signal = max_signal_temp;
    max_bottango_signal_rate_signals_per_seconds = max_signal_rate_signals_per_seconds_temp;
    start_bottango_effector_signal = start_signal_temp;

    effector_added = true;
}

void bottango_manual_sync_effector_position(){
    char identifier_temp[10];
    //NEED TO FINISH THIS.
    
}

void bottango_set_instantcurve(){
    if(!effector_added){
        printf("E_%s, Bottango Effector Not Registered\n",BOTTANGO_SET_INSTANTCURVE_COMMAND);
        return;
    }

    if(put_next_command_word_in_word_buffer() && strcmp(bottango_effector_identifier,word_buffer) == 0){
        
    }
    else{
        printf("E_%s, Recieved Effector Identifier Does Not Match Assigned Identifier\n",BOTTANGO_SET_INSTANTCURVE_COMMAND);
        return;
    }

    if(put_next_command_word_in_word_buffer()){
        int32_t desired_state_variable_to_add;
        if(word_buffer_is_integer_number()){ 
            desired_state_variable_to_add = atoi(word_buffer)*16;
        }
        else if(word_buffer_is_float_number()){
            desired_state_variable_to_add = (int32_t)(atof(word_buffer)*counts_per_bottango_float_signal);
        }
        else{
            printf("E_%s, Recieved Signal is Not a Number\n",BOTTANGO_SET_INSTANTCURVE_COMMAND);
        }
        
        if(queue_get_level(&desired_state_variable_queue)==DESIRED_STATE_VARIABLE_QUEUE_SIZE){
            printf("E%s, State Variable Queue Full\n", BOTTANGO_SET_INSTANTCURVE_COMMAND);
            return;
        }
        else{
            bool queue_add_success = queue_try_add(&desired_state_variable_queue,&desired_state_variable_to_add);
            if(queue_add_success){
                if(debug_flags[CALL_0_ADD_SINGLE_DESIRED_STATE_VARIABLE_COMMAND_NUM] == 1){
                    printf("S%ld, %ld\n", CALL_0_ADD_SINGLE_DESIRED_STATE_VARIABLE_COMMAND_NUM, desired_state_variable_to_add);
                }
                return;
            }
            else{
                printf("E%s, Queue Add Fail\n", BOTTANGO_SET_INSTANTCURVE_COMMAND);
                return;
            }
        }
    }
}

void bottango_set_curve(){

}

void bottango_deregister_effector(){
    if(put_next_command_word_in_word_buffer() && strcmp(bottango_effector_identifier,word_buffer) == 0){
        effector_added = false;
        bottango_effector_identifier[0] = '\0';
    }
}

bool execute_bottango_command(char* command){
    if(strcmp(command,BOTTANGO_SET_INSTANTCURVE_COMMAND) == 0){
        bottango_set_instantcurve();
    }   
    else if(strcmp(command,BOTTANGO_SET_CURVE_COMMAND) == 0){
        bottango_set_curve();
    }    
    else if(strcmp(command,BOTTANGO_TIME_SYNC_COMMAND) == 0){

    } 
    else if(strcmp(command,BOTTANGO_HANDSHAKE_REQUEST_COMMAND) == 0){
        bottango_handshake();
    }
    else if(strcmp(command,BOTTANGO_DEREGISTER_ALL_EFFECTORS_COMMAND) == 0){
        bottango_deregister_effector();
    }
    else if(strcmp(command,BOTTANGO_DEREGISTER_EFFECTOR_COMMAND) == 0){
        bottango_deregister_effector();
    }
    else if(strcmp(command,BOTTANGO_CLEAR_ALL_CURVES_COMMAND) == 0){

    }
    else if(strcmp(command,BOTTANGO_CLEAR_EFFECTOR_CURVES_COMMAND) == 0){

    }
    else if(strcmp(command,BOTTANGO_REGISTER_PIN_SERVO_COMMAND) == 0){
        printf("errEnStep\n"); //THIS LINE OVERLOADS THE STEPPER  NOT ENABLED ERROR
    }
    else if(strcmp(command,BOTTANGO_REGISTER_I2C_SERVO_COMMAND) == 0){
        printf("errEnStep\n"); //THIS LINE OVERLOADS THE STEPPER  NOT ENABLED ERROR
    }
    else if(strcmp(command,BOTTANGO_REGISTER_PIN_STEPPER_COMMAND) == 0){
        printf("errEnStep\n");
    }
    else if(strcmp(command,BOTTANGO_REGISTER_DIR_STEPPER_COMMAND) == 0){
        printf("errEnStep\n");
    }
    else if(strcmp(command,BOTTANGO_REGISTER_I2C_STEPPER_COMMAND) == 0){
        printf("errEnStep\n");
    }
    else if(strcmp(command,BOTTANGO_REGISTER_CURVED_EVENT_COMMAND) == 0){
        printf("errEnStep\n"); //THIS LINE OVERLOADS THE STEPPER  NOT ENABLED ERROR
    }
    else if(strcmp(command,BOTTANGO_REGISTER_ONOFF_EVENT_COMMAND) == 0){
        printf("errEnStep\n"); //THIS LINE OVERLOADS THE STEPPER  NOT ENABLED ERROR
    }
    else if(strcmp(command,BOTTANGO_REGISTER_TRIGGER_EVENT_COMMAND) == 0){
        printf("errEnStep\n"); //THIS LINE OVERLOADS THE STEPPER  NOT ENABLED ERROR
    }
    else if(strcmp(command,BOTTANGO_REGISTER_COLOR_EVENT_COMMAND) == 0){
        printf("errEnStep\n"); //THIS LINE OVERLOADS THE STEPPER  NOT ENABLED ERROR
    }
    else if(strcmp(command,BOTTANGO_REGISTER_CUSTOM_MOTOR_COMMAND) == 0){
        if(effector_added){
            printf("errNoSpaceAvailable\n");
        }
        else{
            bottango_add_effector();
        }
    }
    else if(strcmp(command,BOTTANGO_SET_ONOFFCURVE_COMMAND) == 0){
        printf("%s NOT SUPPORTED\n",BOTTANGO_SET_ONOFFCURVE_COMMAND);
    }
    else if(strcmp(command,BOTTANGO_SET_TRIGGERCURVE_COMMAND) == 0){
        printf("%s NOT SUPPORTED\n",BOTTANGO_SET_TRIGGERCURVE_COMMAND);
    }
    else if(strcmp(command,BOTTANGO_SET_COLOR_CURVE_COMMAND) == 0){
        printf("%s NOT SUPPORTED\n",BOTTANGO_SET_COLOR_CURVE_COMMAND);
    }
    else if(strcmp(command,BOTTANGO_SET_INSTANT_COLOR_CURVE_COMMAND) == 0){
        printf("%s NOT SUPPORTED\n",BOTTANGO_SET_INSTANT_COLOR_CURVE_COMMAND);
    }
    else if(strcmp(command,BOTTANGO_MANUAL_SYNC_COMMAND) == 0){
        bottango_manual_sync_effector_position();
    }
    else{
        return false;
    }
}

/**
 * Scan the USB Buffer for the next command and processes it to completion. If a command is recieved, it blocks non-inturrupting execution until the command is fully recieved and interpreted.
*/
void execute_current_command_in_command_buffer(){
    bool hashResult = checkHash(command_buffer);
    if (hashResult == false){
        printf("%s\n",BOTTANGO_HASH_FAIL_RESPONSE);
        clear_string_from_usb_command_buffer();
    }
    else{ 
        put_next_command_word_in_word_buffer();

        if(word_buffer_is_integer_number()){
            uint32_t command_num = atoi(word_buffer);
            execute_numerical_command(command_num);
        }
        else{
            execute_bottango_command(word_buffer);
        }
        printf("%s\n",BOTTANGO_READY_RESPONSE);
    }
    clear_string_from_usb_command_buffer();
    command_buffer_parsing_index = 0;
}


bool checkHash(char *cmdString)
{
    if (cmdString[0] == '\0')
    {
        return 0;
    }

    char c = cmdString[0];
    int idx = 0;

    // Scan forward to end of string
    while (c != '\0')
    {
        c = cmdString[idx++];
    }
    // Scan backward to find start of hash (don't hash the hash)
    idx -= 1; // One for 'h', one for ','

    while (idx > 0)
    {
        if (cmdString[idx] == ',' && cmdString[idx + 1] == 'h')
        {
            break;
        }
        idx--;
    }

    int hashStartIdx = idx + 2;

    idx -= 1; // For ','

    int hsh = 0;
    while (idx >= 0)
    {
        c = cmdString[idx--];
        hsh += c;
    }

    int ati = atoi(cmdString + hashStartIdx);

    
    if(ati == -1){
        return true; //HASH OVERRIDE
    }

    if (ati == hsh)
    {
        return true; //HASH GOOD
    }
    else
    {
        return false; //HASH BAD
    }

}