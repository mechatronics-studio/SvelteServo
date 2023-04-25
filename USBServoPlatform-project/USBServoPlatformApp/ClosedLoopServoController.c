#include "ClosedLoopServoController.h"
#include "pico/stdlib.h"
#include "RAMVariablesCore1.h"
#include "CoreCommunication.h"
#include "hardware/pwm.h"
#include "macro_header.h"
#include "quadrature_encoder.pio.h"

int32_t error_counts;
int32_t previous_error_counts = 0;
int32_t integrated_error_counts = 0;
int32_t current_state_variable = 0;
int32_t desired_state_variable = 0;

uint16_t control_level;

int32_t error_dot_counts_per_second = 0;
int32_t proportional_control_voltage= 0;
int32_t derivative_control_voltage = 0;
int32_t integral_control_voltage = 0;

int32_t telemetry_reporting_counter = 1;
int32_t desired_state_variable_counter = 1;

int32_t motor_supply_volage_mv;

//telemetry_data telemetry_data_instance = {.time_us = 10, .desired_state_variable_act = 1000};

telemetry_data telemetry_data_report;

bool telemetry_added;

/**
 * This function has not been finished yet. Alo need to add telemetry reporting.
*/
int64_t update_servo_voltage_absolute_position_mode(int32_t alarm_ID, void* empty){
    if(enable_simulated_servo_core1 == 1){
        current_state_variable = (int32_t)simulated_quadrature_counter;
        motor_supply_volage_mv = simulated_servo_voltage_mv_core1;
    }
    else{
        current_state_variable = quadrature_encoder_get_count(pio, sm);
        motor_supply_volage_mv = static_motor_power_supply_voltage_mv_core1;
    }

    if(desired_state_variable_counter == desired_state_update_interval_multiplier_core1){
        queue_try_remove(&desired_state_variable_queue,&desired_state_variable);
        desired_state_variable_counter = 0;
    }
    desired_state_variable_counter++;
    
    error_counts = desired_state_variable - current_state_variable;
    error_dot_counts_per_second = (1000000*(error_counts - previous_error_counts))/(int32_t)servo_update_interval_us_core1_32bit;
    integrated_error_counts = integrated_error_counts + error_counts;
    previous_error_counts = error_counts;

    //ANTI-WINDUP (ONLY INTEGRATE IF THE SYSTEM IS CONVERGING, TOTAL ERROR LESS THAN THRESHOLD)
    if(error_counts > INTEGRATOR_ANTI_WINDUP_THRESHOLD_COUNTS || error_counts < -INTEGRATOR_ANTI_WINDUP_THRESHOLD_COUNTS){
        integrated_error_counts = 0;
    }

    //ANTI-INT32 ROLLOVER, NEED TO FORMALIZE BETTER, BUT I WAS FINDING THAT VERY LARGE STEP CHANGES WITH LARGE ERRORS WOULD CAUSE A ROLLOVER OF THE VOLTAGE.
    /*if(error_counts > 10000){
        error_counts = 10000;
    }
    else if(error_counts < -10000){
         error_counts = -10000;
    }

    if(error_dot_counts_per_second > 1000000){
        error_dot_counts_per_second = 1000000;
    }
    else if(error_dot_counts_per_second < -1000000){
             error_dot_counts_per_second = -1000000;   
    }*/

    //printf("ERROR: %ld, ERROR_DOT: %ld, INTEGRATOR: %ld\n", error_counts, error_dot_counts_per_second, integrated_error_counts);
    proportional_control_voltage = ((int32_t)proportional_gain_core1*error_counts)/1000;
    derivative_control_voltage = ((int32_t)derivative_gain_core1*error_dot_counts_per_second)/1000;
    integral_control_voltage = ((int32_t)integral_gain_core1*integrated_error_counts)/1000; 
    //printf("PROPORTIONAL_VOLTAGE: %ld, DERIVATIVE_VOLTAGE: %ld, INTEGRATOR_VOLTAGE: %ld\n", proportional_control_voltage, derivative_control_voltage, integral_control_voltage);
    
    commanded_control_voltage_mv = proportional_control_voltage + derivative_control_voltage + integral_control_voltage;

    if(commanded_control_voltage_mv > motor_supply_volage_mv){
        commanded_control_voltage_mv = motor_supply_volage_mv;
    }
    if(commanded_control_voltage_mv < -motor_supply_volage_mv){
        commanded_control_voltage_mv = -motor_supply_volage_mv;
    }
  
    if(commanded_control_voltage_mv > 0){
        control_level = (1250*commanded_control_voltage_mv)/motor_supply_volage_mv;
        pwm_set_chan_level(slice_num, channel_num_F, control_level);
        pwm_set_chan_level(slice_num, channel_num_R, 0);
    }
    else if(commanded_control_voltage_mv < 0){
        control_level = (1250*-commanded_control_voltage_mv)/motor_supply_volage_mv;
        pwm_set_chan_level(slice_num, channel_num_F, 0);
        pwm_set_chan_level(slice_num, channel_num_R, control_level);
    }
    else{
        pwm_set_chan_level(slice_num, channel_num_F, 0);
        pwm_set_chan_level(slice_num, channel_num_R, 0);
    }
    
    if(telemetry_reporting_counter == telemetry_reporting_interval_multiplier_core1 && telemetry_reporting_mode_core1 != 0){
        telemetry_data_report.time_us = time_us_32();
        telemetry_data_report.desired_state_variable_des = desired_state_variable;
        telemetry_data_report.desired_state_variable_act = current_state_variable;
        telemetry_data_report.control_voltage_mv = commanded_control_voltage_mv;
        telemetry_added = queue_try_add(&telemetry_output_queue,&telemetry_data_report);
        if(telemetry_added == false){
            printf("%lld CORE1: Critical Error, Telemetry Queue Full\n",time_us_64());
        }
        telemetry_reporting_counter = 0;
    }
    

    //printf("ERROR: %ld, ERROR_DOT: %ld, INTEGRATOR: %ld\n", error_counts, error_dot_counts_per_second, integrated_error_counts);
    //CONSIDER ADDING A TELEMETRY MODE WITH THE ABOVE.

    telemetry_reporting_counter++;
    return servo_update_interval_us_core1_64bit;
}

/**
 * This function has not been finished yet.
*/
int64_t update_servo_voltage_absolute_velocity_mode(int32_t alarm_ID, void* empty){
    /*int32_t error_counts = desired_state_variable - current_state_variable;

    int32_t voltage_commanded = proportional_gain_core1*error_counts;
    previous_error_counts = error_counts;*/
    return servo_update_interval_us_core1_64bit;
}