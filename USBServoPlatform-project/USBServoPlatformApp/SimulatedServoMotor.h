#ifndef SIMULATED_SERVO_MOTOR
#define SIMULATED_SERVO_MOTOR
#include "pico/stdlib.h"


int64_t iterate_simulated_servo(int32_t alarm_ID, void* empty);

void update_calculated_simulated_servo_constants();

extern float forward_voltage;

#endif