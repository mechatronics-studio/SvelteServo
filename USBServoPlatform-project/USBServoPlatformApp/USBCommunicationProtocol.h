#ifndef USB_COMMUNICATION_PROTOCOL
#define USB_COMMUNICATION_PROTOCOL
#include "pico/stdlib.h"
#include <math.h>

#define COMMAND_BUFER_MAX_LENGTH 1000

extern uint32_t usb_command_buffer_string_termination_index;
extern char command_buffer[COMMAND_BUFER_MAX_LENGTH];
extern bool command_read_complete;

void initialize_usb_communication();
void pop_char_from_usb_to_command_buffer();
void clear_string_from_usb_command_buffer();
int64_t transmit_telemetry_data_via_usb(int32_t alarm_ID, void* empty);

#endif