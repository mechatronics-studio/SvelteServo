#ifndef CLOSED_LOOP_SERVO_CONTROLLER
#define CLOSED_LOOOP_SERVO_CONTROLLER
#include "pico/stdlib.h"

int64_t update_servo_voltage_absolute_position_mode(int32_t alarm_ID, void* empty);
int64_t update_servo_voltage_absolute_velocity_mode(int32_t alarm_ID, void* empty);

#endif