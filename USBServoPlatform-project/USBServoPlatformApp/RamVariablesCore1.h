#ifndef RAM_VARIABLES_CORE1
#define RAM_VARIABLES_CORE1
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"

//These variables are duplicated accross both Core1 and Core0 (local copies for thread protection)

extern uint8_t servo_enable_core1;
extern uint8_t control_mode_core1;

extern uint32_t static_motor_power_supply_voltage_mv_core1;

extern uint8_t enable_simulated_servo_core1;

extern uint32_t simulated_servo_update_interval_us_core1_32bit;
extern uint64_t simulated_servo_update_interval_us_core1_64bit;
extern float simulated_servo_update_interval_s_core1;

extern uint32_t simulated_servo_voltage_mv_core1;
extern float simulated_servo_torque_constant_Nm_per_amp_core1;
extern float simulated_servo_winding_resistance_ohms_core1;
extern uint32_t simulated_servo_counts_per_revolution_core1;
extern float simulated_servo_inertia_kg_m2_core1;
extern float simulated_servo_damping_drag_coefficient_Nm_sec_per_rad_core1;
extern float simulated_servo_kinetic_friction_torque_Nm_core1;

extern uint32_t proportional_gain_core1;
extern uint32_t derivative_gain_core1;
extern uint32_t integral_gain_core1;

extern uint32_t servo_update_interval_us_core1_32bit;
extern uint64_t servo_update_interval_us_core1_64bit;

extern uint32_t desired_state_update_interval_multiplier_core1;

extern uint8_t telemetry_reporting_mode_core1;
extern uint32_t telemetry_reporting_interval_multiplier_core1;

extern uint8_t LED_mode_core1;

extern int32_t commanded_control_voltage_mv;

//core1 RAM Variables and Functions

extern float simulated_quadrature_counter;
extern float simulated_quadrature_counter_dot;

//Global Variable Declarations
extern alarm_pool_t* core1_alarm_pool;
extern alarm_id_t servo_loop_interrupt;
extern alarm_id_t simulated_servo_interrupt;

extern PIO pio;
extern const uint sm;

extern uint slice_num;
extern uint channel_num_F;
extern uint channel_num_R;


void initialize_RAM_variables_core1();

#endif