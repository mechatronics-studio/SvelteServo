#include "pico/stdlib.h"
#include "RamVariablesCore1.h"
#include "math.h"
#ifndef PI
#define PI 3.14159265359
#endif
#include "macro_header.h"
#include <stdio.h>

//All Simulated Servo Values Default to Zero
float forward_constant_counts_per_sec_change_per_volt = 0; 
float damping_constant_counts_per_sec_change_per_counts_per_sec = 0; 
float friction_constant_counts_per_sec_change = 0;

float friction_torque = 0;

float kinetic_friction_threshold_speed_counts_per_sec;

/**
 * This function progresses the simulated servo motor by one time step. 
*/
int64_t iterate_simulated_servo(int32_t alarm_ID, void* empty){

    if(simulated_quadrature_counter_dot > kinetic_friction_threshold_speed_counts_per_sec){
        friction_torque = -friction_constant_counts_per_sec_change;
    }
    else if(simulated_quadrature_counter_dot < -kinetic_friction_threshold_speed_counts_per_sec){
        friction_torque = friction_constant_counts_per_sec_change;
    }

    friction_torque = 0.0;
    damping_constant_counts_per_sec_change_per_counts_per_sec = 1.0;

    simulated_quadrature_counter = simulated_quadrature_counter + simulated_quadrature_counter_dot*simulated_servo_update_interval_s_core1;
    simulated_quadrature_counter_dot = (1.0-damping_constant_counts_per_sec_change_per_counts_per_sec)*simulated_quadrature_counter_dot + forward_constant_counts_per_sec_change_per_volt*((float)commanded_control_voltage_mv)/1000.0 - friction_torque;

    /*printf("t1: %e, t2: %ld, t3: %f, t4: %e, t5: %e\n", forward_constant_counts_per_sec_change_per_volt, commanded_control_voltage_mv, (float)commanded_control_voltage_mv, (float)-24000, forward_constant_counts_per_sec_change_per_volt*((float)commanded_control_voltage_mv)/1000.0);

    #ifdef MASTER_DEBUG
        printf("%lld Core1: simulated_quadrature_counter: %e simulated_quadrature_counter_dot %e\n", time_us_64() ,simulated_quadrature_counter, simulated_quadrature_counter_dot);
    #endif*/
    return simulated_servo_update_interval_us_core1_64bit;
}

void update_calculated_simulated_servo_constants(){
    forward_constant_counts_per_sec_change_per_volt = simulated_servo_torque_constant_Nm_per_amp_core1*(float)simulated_servo_counts_per_revolution_core1*simulated_servo_update_interval_s_core1/(2*PI*simulated_servo_inertia_kg_m2_core1*simulated_servo_winding_resistance_ohms_core1);
    #ifdef MASTER_DEBUG
    printf("%lld Core1: Forward constant was recalculated to %e (counts_change)/V\n", time_us_64() ,forward_constant_counts_per_sec_change_per_volt);
    #endif


    damping_constant_counts_per_sec_change_per_counts_per_sec = (simulated_servo_torque_constant_Nm_per_amp_core1*simulated_servo_torque_constant_Nm_per_amp_core1/(simulated_servo_inertia_kg_m2_core1*simulated_servo_winding_resistance_ohms_core1)+(simulated_servo_damping_drag_coefficient_Nm_sec_per_rad_core1/simulated_servo_inertia_kg_m2_core1))*simulated_servo_update_interval_s_core1;
    #ifdef MASTER_DEBUG
    printf("%lld Core1: Damping constant was recalculated to %e (counts_change)/(counts/sec)\n", time_us_64(), damping_constant_counts_per_sec_change_per_counts_per_sec);
    #endif

    damping_constant_counts_per_sec_change_per_counts_per_sec = (float)simulated_servo_counts_per_revolution_core1*simulated_servo_kinetic_friction_torque_Nm_core1*simulated_servo_update_interval_s_core1/(2*PI*simulated_servo_inertia_kg_m2_core1);
    #ifdef MASTER_DEBUG
    printf("%lld Core1: Friction constant was recalculated to %e (counts_change)\n", time_us_64(), friction_constant_counts_per_sec_change);
    #endif

    kinetic_friction_threshold_speed_counts_per_sec = (KINETIC_FRICTION_SPEED_THRESHOLD_REV_PER_MIN*simulated_servo_counts_per_revolution_core1)/60.0;
}

