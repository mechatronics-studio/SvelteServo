#include "USBCommunicationProtocol.h"
#include "pico/stdlib.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "CoreCommunication.h"
#include <math.h>
#include "macro_header.h"
#include "RamVariablesCore0.h"

#define EMPTY_CHAR -1 ///<This defines the value that corresponds to an empty buffer

char command_buffer[COMMAND_BUFER_MAX_LENGTH];
uint32_t usb_command_buffer_string_termination_index = 0;
int8_t peeked_char = EMPTY_CHAR; ///<This variable is a buffer for getting and storing the first character from the USB buffer to provide peeking functionality.
int32_t integer_buffer; ///<The memory buffer that stores the recieved word after it is converted to an integer
bool command_read_complete = false;

int64_t transmit_telemetry_data_via_usb(int32_t alarm_ID, void* empty){
    if(queue_get_level(&telemetry_output_queue) > 0){
        telemetry_data recieved_telemetry_data;
        queue_remove_blocking(&telemetry_output_queue,&recieved_telemetry_data);
        switch(telemetry_reporting_mode_core0){
        case 0:
            break;
        case 1:
            printf("/*%ld,%ld*/\n", recieved_telemetry_data.time_us, recieved_telemetry_data.control_voltage_mv);
            break;
        case 2:
            printf("/*%ld,%ld*/\n", recieved_telemetry_data.time_us, recieved_telemetry_data.desired_state_variable_act);
            break;
        case 3:
            printf("/*%ld,%ld,%ld,%ld*/\n", recieved_telemetry_data.time_us, recieved_telemetry_data.desired_state_variable_des, recieved_telemetry_data.desired_state_variable_act, recieved_telemetry_data.control_voltage_mv);
            break;
        }
    }
    return telemetry_reporting_interval_us;
}


/**
 * THIS IS CURRENTLY BROKEN. NEED TO ADD CALLS FOR ENABLING SIMULATED SERVO AND SERVO ENABLE FROM DEFAULTS.
*/
void initialize_usb_communication(){
    stdio_init_all();
    sleep_ms(3000); //On Start, give 3 seconds for an autoconnect.
}

/**
 * If the 'peeked_char' global variable is empty, pop the next character off the USB buffer and store it in 'peeked_char', otherwise, return the already populated 'peeked_char'.
 * @return peaked_charr
*/
int peek_char() {
  if(peeked_char == EMPTY_CHAR) {
    peeked_char = getchar_timeout_us(0);
  }
  return peeked_char;
}

/**
 * Peek the next character and pop it off.
 * @return popped_char
*/
char get_char() {
  int popped_char = peek_char();
  peeked_char = EMPTY_CHAR;
  return popped_char;
}

/**
 * Pop the next char off the USB buffer and transfer it to the command buffer.
 * * If no chars exist on the USB buffer, nothing happens.
 * * If a full command has already been recieved, nothing happens.
*/
void pop_char_from_usb_to_command_buffer(){
    if(command_read_complete == false){
        peek_char();
        if(peeked_char != EMPTY_CHAR && peeked_char != '\n'){
            command_buffer[usb_command_buffer_string_termination_index] = get_char();
            usb_command_buffer_string_termination_index++;
            command_buffer[usb_command_buffer_string_termination_index] = '\0';
        }
        else if(peeked_char == '\n'){
            get_char();
            command_read_complete = true;
        }
    }
}

/**
 * Clears the command buffer and sets the command_read_complete flag to false.
*/
void clear_string_from_usb_command_buffer(){
    command_read_complete = false;
    command_buffer[0] = '\0';
    usb_command_buffer_string_termination_index = 0;
}