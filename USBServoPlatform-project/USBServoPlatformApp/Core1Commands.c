#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <string.h>
#include <stdio.h>
#include "CoreCommunication.h"
#include "LEDController.h"
#include "RamVariablesCore1.h"
#include "ClosedLoopServoController.h"
#include "SimulatedServoMotor.h"
#include "macro_header.h"
#include "MT6701Encoder.h"

/**
 * This function modifies the core1 copy of the servo_enable RAM Variable and also enables or disables the loop function corresponding to the servo loop.
 * @param servo_enable A booleon integer value corresponding to whether the servo is enabled or disabled (1 for true, 0 for false).
*/
void set_servo_enable(int32_t servo_enable){
    if(servo_enable != 1){
        servo_enable = 0;
    }
    if(servo_enable_core1 != servo_enable){
        servo_enable_core1 = servo_enable;
        if(servo_enable_core1 == 1){
            if(control_mode_core1 == 1){
                servo_loop_interrupt = alarm_pool_add_alarm_in_us(core1_alarm_pool, servo_update_interval_us_core1_64bit, update_servo_voltage_absolute_position_mode, NULL, false);
            }
            else if(control_mode_core1 == 2){
                servo_loop_interrupt = alarm_pool_add_alarm_in_us(core1_alarm_pool, servo_update_interval_us_core1_64bit, update_servo_voltage_absolute_velocity_mode, NULL, false);
            }
            set_LED_mode(3, core1_alarm_pool);
            #ifdef MASTER_DEBUG
                printf("%lld Core1: Servo control has been enabled, control mode is %ld.\n", time_us_64(), control_mode_core1);
            #endif
        }
        else{
            alarm_pool_cancel_alarm(core1_alarm_pool, servo_loop_interrupt);
            set_LED_mode(2, core1_alarm_pool);
            //SET SERVO OUTPUT TO ZERO
            commanded_control_voltage_mv = 0;
            
            #ifdef MASTER_DEBUG
            printf("%lld Core1: Servo control has been disabled\n", time_us_64());
            #endif
        }
    }
}

/**
 * If the servo_enable is 1, this function restarts the servo by disabilng and then re-enabling it. Otherwise, the servo is left disabled.
*/
void restart_servo(){
    if(servo_enable_core1 == 1){
    set_servo_enable(0);
    set_servo_enable(1);
    }
}

/**
 * This function modifies the core1 copy of the servo_update_interval_us ram variable (core1 32bit and 64bit copies).
 * @param servo_loop_interval_dms The new servo update interval, in decimilliseconds (not microseconds).
*/
void set_servo_loop_interval(int32_t servo_loop_interval_dms){
    if(servo_loop_interval_dms < 1){
        servo_loop_interval_dms = -servo_loop_interval_dms;
    }
    if(servo_loop_interval_dms == 0){
        servo_loop_interval_dms = 1;
    }
    if(servo_update_interval_us_core1_32bit != servo_loop_interval_dms*100){
    servo_update_interval_us_core1_32bit = servo_loop_interval_dms*100;
    servo_update_interval_us_core1_64bit = servo_loop_interval_dms*100;
    }
}

/**
 * This function modifies the core1 copy of the desired_state_variable_update_interval_multiplier ram variable.
 * @param desired_state_update_interval_multiplier The new desired state variable update interval multiplier, which defines the number of servo loops before the next desired state variable is popped off the desired state variable queue.
*/
void set_desired_state_variable_update_interval_multiplier(int32_t desired_state_update_interval_multiplier){
    if(desired_state_update_interval_multiplier < 1){
        desired_state_update_interval_multiplier = -desired_state_update_interval_multiplier;
    }
    if(desired_state_update_interval_multiplier == 0){
        desired_state_update_interval_multiplier = 1;
    }
    if(desired_state_update_interval_multiplier_core1 != desired_state_update_interval_multiplier){
    desired_state_update_interval_multiplier_core1 = desired_state_update_interval_multiplier;
    }
}

/**
 * This function modifies the core1 copy of the control_mode ram variable.
 * @param control_mode The new desired state variable update interval multiplier, which defines the number of servo loops before the next desired state variable is popped off the desired state variable queue.
 * @return A boolean the describes whether the control mode was changed and the servo loop needs to be restarted.
*/
bool set_control_mode(int32_t control_mode){
    if(control_mode != 1 && control_mode !=2 ){
        control_mode = 1;
    }
    if(control_mode_core1 != control_mode){
    control_mode_core1 = control_mode;
    return true;
    }
    return false;
}

/**
 * This function modifies the core1 copy of the proportional_gain ram variable.
 * @param proportional_gain The new desired state variable update interval multiplier, which defines the number of servo loops before the next desired state variable is popped off the desired state variable queue.
*/
void set_proportional_gain(int32_t proportional_gain){
    if(proportional_gain < 0){
        proportional_gain = 0;
    }
    if(proportional_gain_core1 != proportional_gain){
    proportional_gain_core1 = proportional_gain;
    }
}

/**
 * This function modifies the core1 copy of the derivative_gain ram variable.
 * @param derivative_gain The new desired state variable update interval multiplier, which defines the number of servo loops before the next desired state variable is popped off the desired state variable queue.
*/
void set_derivative_gain(int32_t derivative_gain){
    if(derivative_gain < 0){
        derivative_gain = 0;
    }
    if(derivative_gain_core1 != derivative_gain){
    derivative_gain_core1 = derivative_gain;
    }
}

/**
 * This function modifies the core1 copy of the integral_gain ram variable.
 * @param integral_gain The new desired state variable update interval multiplier, which defines the number of servo loops before the next desired state variable is popped off the desired state variable queue.
*/
void set_integral_gain(int32_t integral_gain){
    if(integral_gain < 0){
        integral_gain = 0;
    }
    if(integral_gain_core1 != integral_gain){
    integral_gain_core1 = integral_gain;
    }
}

/**
 *  This function modifies the core1 copy of the simulated_servo_enable RAM Variable. This will enable or disable the simulated servo loop and modify the measured state variables to be real or simulated accordingly.
 * @param simulated_servo_enable 
*/
void set_simulated_servo_enable(int32_t simulated_servo_enable){
    if(simulated_servo_enable != 1){
        simulated_servo_enable = 0;
    }
    if(enable_simulated_servo_core1 != simulated_servo_enable){
        if(simulated_servo_enable == 1){
            update_calculated_simulated_servo_constants(); 
            float simulated_quadrature_counter = 0;
            float simulated_quadrature_counter_dot = 0;
            simulated_servo_interrupt = alarm_pool_add_alarm_in_us(core1_alarm_pool, simulated_servo_update_interval_us_core1_64bit, iterate_simulated_servo, NULL, false);
        }
        else{
            alarm_pool_cancel_alarm(core1_alarm_pool, simulated_servo_interrupt);
        }
        enable_simulated_servo_core1 = simulated_servo_enable;
    }
}

/**
 * This function modifies the core1 copy of the simulated_servo_update_interval_us ram variable (core1 32bit and 64bit copies).
 * @param simulated_servo_loop_interval_dms The new servo update interval, in decimilliseconds (not microseconds).
*/
void set_simulated_servo_loop_interval(int32_t simulated_servo_loop_interval_dms){
    if(simulated_servo_loop_interval_dms < 1){
        simulated_servo_loop_interval_dms = -simulated_servo_loop_interval_dms;
    }
    if(simulated_servo_loop_interval_dms == 0){
        simulated_servo_loop_interval_dms = 1;
    }

    int32_t simulated_servo_loop_interval_us = simulated_servo_loop_interval_dms*100;
    float simulated_servo_update_interval_s = ((float)simulated_servo_loop_interval_us)/1000000;

    if(simulated_servo_update_interval_us_core1_32bit != simulated_servo_loop_interval_us){
    simulated_servo_update_interval_us_core1_32bit = simulated_servo_loop_interval_us;
    simulated_servo_update_interval_us_core1_64bit = simulated_servo_loop_interval_us;
    simulated_servo_update_interval_s_core1 = simulated_servo_update_interval_s;
    }
}

/**
 * This function modifies the core1 copy of the simulated_servo_voltage_mv_core1 ram variable.
 * @param simulated_servo_voltage_mv_core1
*/
void set_simulated_servo_voltage_mv(int32_t simulated_servo_voltage_mv){
    if(simulated_servo_voltage_mv < 1){
        simulated_servo_voltage_mv = -simulated_servo_voltage_mv;
    }
    if(simulated_servo_voltage_mv == 0){
        simulated_servo_voltage_mv = 1;
    }
    if(simulated_servo_voltage_mv_core1 != simulated_servo_voltage_mv){
    simulated_servo_voltage_mv_core1 = simulated_servo_voltage_mv;
    }
}

/**
 * This function modifies the core1 copy of the simulated_servo_torque_constant_Nm_per_amp_core1 ram variable.
 * @param simulated_servo_torque_constant_Nm_per_amp 
*/
void set_simulated_servo_torque_constant_Nm_per_amp(float simulated_servo_torque_constant_Nm_per_amp){
    if(simulated_servo_torque_constant_Nm_per_amp < 0){
        simulated_servo_torque_constant_Nm_per_amp = 0;
    }

    if(simulated_servo_torque_constant_Nm_per_amp_core1 != simulated_servo_torque_constant_Nm_per_amp){
    simulated_servo_torque_constant_Nm_per_amp_core1 = simulated_servo_torque_constant_Nm_per_amp;
    }
}

/**
 * This function modifies the core1 copy of the simulated_servo_winding_resistance_ohms ram variable.
 * @param simulated_servo_winding_resistance_ohms
*/
void set_simulated_servo_winding_resistance_ohms(float simulated_servo_winding_resistance_ohms){
    if(simulated_servo_winding_resistance_ohms < MINIMUM_SIMULATED_SERVO_WINDING_RESISTANCE_OHMS){
        simulated_servo_winding_resistance_ohms = MINIMUM_SIMULATED_SERVO_WINDING_RESISTANCE_OHMS;
    }

    if(simulated_servo_winding_resistance_ohms_core1 != simulated_servo_winding_resistance_ohms){
    simulated_servo_winding_resistance_ohms_core1 = simulated_servo_winding_resistance_ohms;
    }
}

/**
 * This function modifies the core1 copy of the simulated_servo_counts_per_revolution ram variable.
 * @param simulated_servo_counts_per_revolution 
*/
void set_simulated_servo_counts_per_revolution(int32_t simulated_servo_counts_per_revolution){
    if(simulated_servo_counts_per_revolution < 1){
        simulated_servo_counts_per_revolution = 1;
    }

    if(simulated_servo_counts_per_revolution_core1 != simulated_servo_counts_per_revolution){
    simulated_servo_counts_per_revolution_core1 = simulated_servo_counts_per_revolution;
    }
}

/**
 * This function modifies the core1 copy of the simulated_servo_inertia_kg_m2 ram variable. Note the conversion factor is 1/10000000000.
 * @param simulated_servo_inertia_kg_m2 The simulated servo inertia in milligrams*cm^2 (not kg*m^2) 
*/
void set_simulated_servo_inertia_kg_m2(float simulated_servo_inertia_kg_m2){
    if(simulated_servo_inertia_kg_m2 < MINIMUM_SIMULATED_SERVO_INERTIA_KG_M2){
        simulated_servo_inertia_kg_m2 = MINIMUM_SIMULATED_SERVO_INERTIA_KG_M2;
    }

    if(simulated_servo_inertia_kg_m2_core1 != simulated_servo_inertia_kg_m2){
    simulated_servo_inertia_kg_m2_core1 = simulated_servo_inertia_kg_m2;
    }
}

/**
 * This function modifies the core1 copy of the simulated_servo_damping_drag_coefficient_Nm_sec_per_rad ram variable. Note the conversion factor is 1/1000000000.
 * @param simulated_servo_damping_drag_coefficient_nNm_sec_per_rad The new simulated damping drag coefficient in nanotnewton*meters/(rad/s) (not Nm per rad/s)
*/
void set_simulated_servo_damping_drag_coefficient_Nm_sec_per_rad(float simulated_servo_damping_drag_coefficient_Nm_sec_per_rad){
    if(simulated_servo_damping_drag_coefficient_Nm_sec_per_rad < 1){
        simulated_servo_damping_drag_coefficient_Nm_sec_per_rad = 0;
    }

    if(simulated_servo_damping_drag_coefficient_Nm_sec_per_rad_core1 != simulated_servo_damping_drag_coefficient_Nm_sec_per_rad){
    simulated_servo_damping_drag_coefficient_Nm_sec_per_rad_core1 = simulated_servo_damping_drag_coefficient_Nm_sec_per_rad;
    }
}

/**
 * This function modifies the core1 copy of the simulated_servo_kinetic_friction_torque_Nm_core1 ram variable. Note the conversion factor is 1/1000000000.
 * @param  simulated_servo_kinetic_friction_torque_nNm The new simulated kinetic friction torque in nanonewton*meters (not newtonmeters)
*/
void set_simulated_servo_kinetic_friction_torque_Nm(float simulated_servo_kinetic_friction_torque_Nm){
    if(simulated_servo_kinetic_friction_torque_Nm < 0){
        simulated_servo_kinetic_friction_torque_Nm = 0;
    }

    if(simulated_servo_kinetic_friction_torque_Nm_core1 != simulated_servo_kinetic_friction_torque_Nm){
    simulated_servo_kinetic_friction_torque_Nm_core1 = simulated_servo_kinetic_friction_torque_Nm;
    }
}

/**
 * This function modifies the core1 copy of the static_motor_power_supply_voltage_mv ram variable.
 * @param static_motor_power_supply_voltage_mv 
*/
void set_static_motor_power_supply_voltage_mv(int32_t static_motor_power_supply_voltage_mv){
    if(static_motor_power_supply_voltage_mv_core1 != static_motor_power_supply_voltage_mv){
    static_motor_power_supply_voltage_mv_core1 = static_motor_power_supply_voltage_mv;
    }
}

/**
 * This function modifies the core1 copy of the telemetry_reporting_mode ram variable.
 * @param telemetry_reporting_mode 
*/
void set_telemetry_reporting_mode(int32_t telemetry_reporting_mode){
    if(telemetry_reporting_mode != 1 && telemetry_reporting_mode != 2 && telemetry_reporting_mode != 3){
        telemetry_reporting_mode = 0;
    }
    if(telemetry_reporting_mode_core1 != telemetry_reporting_mode){
    telemetry_reporting_mode_core1 = telemetry_reporting_mode;
    }
}


/**
 * This function modifies the core1 copy of the telemetry_reporting_interval_multiplier_core1 ram variable.
 * @param telemetry_reporting_interval_multiplier_core1 The new desired state variable update interval multiplier, which defines the number of servo loops before the next desired state variable is popped off the desired state variable queue.
*/
void set_telemetry_reporting_interval_multiplier(int32_t telemetry_reporting_interval_multiplier){
    if(telemetry_reporting_interval_multiplier <= 0){
        telemetry_reporting_interval_multiplier = 1;
    }
    if(telemetry_reporting_interval_multiplier_core1 != telemetry_reporting_interval_multiplier){
    telemetry_reporting_interval_multiplier_core1 = telemetry_reporting_interval_multiplier;
    }
}


void get_command_from_core0(){
    #ifdef MASTER_DEBUG
        //printf("Core1: Looking for Command from Core0");
    #endif
    if(queue_get_level(&core1_command_queue) > 0){
        core_command_response recieved_command;
        queue_remove_blocking(&core1_command_queue,&recieved_command);
        uint8_t recieved_executive_call = recieved_command.executive_call;
        #ifdef MASTER_DEBUG
        printf("%lld Core1: Recieved Executive Command %i from Core0\n", time_us_64(), recieved_executive_call);
        #endif

        int32_t recieved_supporting_input_integer = recieved_command.supporting_input_integer;
        float recieved_supporting_input_float = recieved_command.supporting_input_float;
        switch (recieved_executive_call){
        case 0: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break;

        case 1: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        case 2: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        case 3: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        case 4: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break;

        case 5: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        case 6: //Enable or Disable Servo Motor
            set_servo_enable(recieved_supporting_input_integer);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of servo_enable changed to %ld\n", time_us_64(), servo_enable_core1);
            #endif
            send_response_to_core_0(recieved_executive_call,1,1);
            break; 

        case 7: //Set Servo Loop Interval
            set_servo_loop_interval(recieved_supporting_input_integer);
            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of servo_loop_interval_us changed to %ld\n", time_us_64(), servo_update_interval_us_core1_32bit);
            #endif
            send_response_to_core_0(recieved_executive_call,1,1);
            break; 

        case 8: //Set Desired State Variable Update Interval Multiplier
            set_desired_state_variable_update_interval_multiplier(recieved_supporting_input_integer);
            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of desired_state_variable_update_interval_multiplier changed to %ld\n", time_us_64(), desired_state_update_interval_multiplier_core1);
            #endif
            send_response_to_core_0(recieved_executive_call,1,1);
            break;

        case 9:{ //Set Motion Control Mode
            bool control_mode_modified = set_control_mode(recieved_supporting_input_integer);
            if(control_mode_modified){
                restart_servo();
            }

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of control_mode changed to %ld\n", time_us_64(), control_mode_core1);
            #endif
            send_response_to_core_0(recieved_executive_call,1,1);
            break; 
        }
        case 10: //Set Proportional Gain
            set_proportional_gain(recieved_supporting_input_integer);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of proportional_gain changed to %ld\n", time_us_64(), proportional_gain_core1);
            #endif
            send_response_to_core_0(recieved_executive_call,1,1);
            break; 

        case 11: //Set Derivative Gain
            set_derivative_gain(recieved_supporting_input_integer);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of derivative_gain changed to %ld\n", time_us_64(), derivative_gain_core1);
            #endif
            send_response_to_core_0(recieved_executive_call,1,1);
            break; 

        case 12: //Set Integral Gain
            set_integral_gain(recieved_supporting_input_integer);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of integral_gain changed to %ld\n", time_us_64(), integral_gain_core1);
            #endif
            send_response_to_core_0(recieved_executive_call,1,1);
            break;

        case 13: //Reserved
            send_response_to_core_0(recieved_executive_call,0,1);
            break; 

        case 14: //Reserved
            send_response_to_core_0(recieved_executive_call,0,1);
            break; 

        case 15: //Reserved
            send_response_to_core_0(recieved_executive_call,0,1);
            break; 

        case 16: //Reserved
            send_response_to_core_0(recieved_executive_call,0,1);
            break;

        case 17: //Reserved
            send_response_to_core_0(recieved_executive_call,0,1);
            break; 

        case 18: //Reserved
            send_response_to_core_0(recieved_executive_call,0,1);
            break; 

        case 19: //Reserved
            send_response_to_core_0(recieved_executive_call,0,1);
            break; 

        case 20: //Reserved
            send_response_to_core_0(recieved_executive_call,0,1);
            break;

        case 21: //Enable Simulated Servo
            set_simulated_servo_enable(recieved_supporting_input_integer);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of enable_simulated_servo changed to %ld\n", time_us_64(), enable_simulated_servo_core1);
            #endif
            send_response_to_core_0(recieved_executive_call,1,1);
            break; 

        case 22: //Set Simulated Servo Update Interval
            set_simulated_servo_loop_interval(recieved_supporting_input_integer);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of simulated_servo_update_interval_us changed to %ld\n", time_us_64(), simulated_servo_update_interval_us_core1_32bit);
            #endif
            update_calculated_simulated_servo_constants();
            send_response_to_core_0(recieved_executive_call,1,1);
            break; 

        case 23: //Set Simulated Servo Voltage
            set_simulated_servo_voltage_mv(recieved_supporting_input_integer);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of simulated_servo_voltage_mv changed to %ld\n", time_us_64(), simulated_servo_voltage_mv_core1);
            #endif
            send_response_to_core_0(recieved_executive_call,1,1);
            break; 

        case 24: //Set Simulated Servo Torque Constant
            set_simulated_servo_torque_constant_Nm_per_amp(recieved_supporting_input_float);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of simulated_servo_torque_constant_Nm_per_amp changed to %0.9f\n", time_us_64(), simulated_servo_torque_constant_Nm_per_amp_core1);
            #endif
            update_calculated_simulated_servo_constants();
            send_response_to_core_0(recieved_executive_call,1,1);
            break;

        case 25: //Set Simulated Servo Winding Resistance
            set_simulated_servo_winding_resistance_ohms(recieved_supporting_input_float);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of simulated_servo_winding_resistance_ohms changed to %0.9f\n", time_us_64(), simulated_servo_winding_resistance_ohms_core1);
            #endif
            update_calculated_simulated_servo_constants();
            send_response_to_core_0(recieved_executive_call,1,1);
            break; 

        case 26: //Set Simulated Servo Counts Per Revolution
            set_simulated_servo_counts_per_revolution(recieved_supporting_input_integer);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of simulated_servo_counts_per_revolution changed to %ld\n", time_us_64(), simulated_servo_counts_per_revolution_core1);
            #endif
            update_calculated_simulated_servo_constants();
            send_response_to_core_0(recieved_executive_call,1,1);
            break; 

        case 27: //Set Simulated Servo Inertia
            set_simulated_servo_inertia_kg_m2(recieved_supporting_input_float);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of simulated_servo_inertia_kg_m2 changed to %0.9f\n", time_us_64(), simulated_servo_inertia_kg_m2_core1);
            #endif
            update_calculated_simulated_servo_constants();
            send_response_to_core_0(recieved_executive_call,1,1);
            break; 

        case 28: //Set Simulated Servo Damping Coefficient
            set_simulated_servo_damping_drag_coefficient_Nm_sec_per_rad(recieved_supporting_input_float);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of simulated_servo_damping_drag_coefficient_Nm_sec_per_rad changed to %0.9f\n", time_us_64(), simulated_servo_damping_drag_coefficient_Nm_sec_per_rad_core1);
            #endif
            update_calculated_simulated_servo_constants();
            send_response_to_core_0(recieved_executive_call,1,1);
            break; 

        case 29: //Set Simulated Servo Static Friction Torque
            set_simulated_servo_kinetic_friction_torque_Nm(recieved_supporting_input_float);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of simulated_servo_kinetic_friction_torque_Nm changed to %0.9f\n", time_us_64(), simulated_servo_kinetic_friction_torque_Nm_core1);
            #endif
            update_calculated_simulated_servo_constants();
            send_response_to_core_0(recieved_executive_call,1,1);
            break; 

        case 30: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break;

        case 31: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        case 32: //Measure Motor Power Supply Voltage (Still to be implemented)
            break; 

        case 33: //Set Static Motor Power Supply Voltage
            set_static_motor_power_supply_voltage_mv(recieved_supporting_input_integer);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of static_motor_power_supply_voltage_mv changed to %ld\n", time_us_64(), static_motor_power_supply_voltage_mv_core1);
            #endif
            send_response_to_core_0(recieved_executive_call,1,1);
            break; 

        case 34: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        case 35: //Set Position in Counts STILL NEEDS TO BE IMPLEMENTED
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        case 36: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break;

        case 37: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        case 38: //Set Telemetry Reporting Mode
            set_telemetry_reporting_mode(recieved_supporting_input_integer);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of telemetry_reporting_mode changed to %ld\n", time_us_64(), telemetry_reporting_mode_core1);
            #endif
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        case 39: //Set Telemetry Reporting Interval Multiplier
            set_telemetry_reporting_interval_multiplier(recieved_supporting_input_integer);

            #ifdef MASTER_DEBUG
            printf("%lld Core1: Core1 copy of telemetry_reporting_interval_multiplier changed to %ld\n", time_us_64(), telemetry_reporting_interval_multiplier_core1);
            #endif
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        case 40: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break;

        case 41: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        case 42: //Set Error Voltage Limit STILL NEEDS TO BE IMPLEMENTED
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        case 43: //Set Position Error Limit STILL NEEDS TO BE IMPLEMENTED
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        case 44: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break;

        case 45: //Set Velocity Error Limit STILL NEEDS TO BE IMPLEMENTED
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        case 46: //Reserved
            send_response_to_core_0(recieved_executive_call,0,0);
            break; 

        }
    }
}

void setup_core1_peripherals(){
    initialize_RAM_variables_core1();
    #ifdef MASTER_DEBUG
    printf("%lld Core1: Ram Variables Initialized.\n", time_us_64());
    #endif

    initialize_LED();
    #ifdef MASTER_DEBUG
    printf("%lld Core1: LED Initialized.\n", time_us_64());
    #endif

    initialize_MT6701();
    #ifdef MASTER_DEBUG
    printf("%lld Core1: Enocder Initialized.\n", time_us_64());
    #endif
    
    set_LED_mode(3,core1_alarm_pool);

}