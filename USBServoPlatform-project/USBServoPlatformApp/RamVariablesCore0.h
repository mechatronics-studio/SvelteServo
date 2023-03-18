#ifndef RAM_VARIABLES_CORE0
#define RAM_VARIABLES_CORE0
#include "pico/stdlib.h"
#include "BezierCurve.h"

//These variables are duplicated accross both Core1 and Core0 (local copies for thread protection)

extern uint8_t servo_enable_core0;
extern uint8_t control_mode_core0;

extern uint32_t static_motor_power_supply_voltage_mv_core0;

extern uint8_t enable_simulated_servo_core0;

extern uint32_t simulated_servo_update_interval_us_core0;

extern uint32_t simulated_servo_voltage_mv_core0;
extern float simulated_servo_torque_constant_Nm_per_amp_core0;
extern float simulated_servo_winding_resistance_ohms_core0;
extern uint32_t simulated_servo_counts_per_revolution_core0;
extern float simulated_servo_inertia_kg_m2_core0;
extern float simulated_servo_damping_drag_coefficient_Nm_sec_per_rad_core0;
extern float simulated_servo_kinetic_friction_torque_Nm_core0;

extern uint32_t proportional_gain_core0;
extern uint32_t derivative_gain_core0;
extern uint32_t integral_gain_core0;

extern uint32_t servo_update_interval_us_core0;

extern uint32_t desired_state_update_interval_multiplier_core0;

extern uint8_t telemetry_reporting_mode_core0;
extern uint32_t telemetry_reporting_interval_multiplier_core0;

extern uint8_t LED_mode_core0;

//core0 Unique RAM Variables and Functions

extern float physical_gear_ratio_counts_per_axis_output_unit;
extern float simulated_gear_ratio_counts_per_axis_output_unit;

extern uint64_t host_time_us_at_last_sync;
extern int64_t  device_time_minus_host_time_us;

extern uint64_t telemetry_reporting_interval_us;

extern uint8_t debug_flags[47];

extern alarm_pool_t* core0_alarm_pool;

extern bool bottango_session_connected;

extern int32_t counts_per_bottango_int_signal;
extern float counts_per_bottango_float_signal;

void initiate_RAM_variables_core0();

#endif