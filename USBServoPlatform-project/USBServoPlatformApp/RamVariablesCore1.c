#include "RamVariablesCore1.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "macro_header.h"
#include "hardware/pwm.h"

#define MOTOR_CONTROL_PIN_1 20
#define MOTOR_CONTROL_PIN_2 21

uint8_t servo_enable_core1 = 0;
uint8_t control_mode_core1 = DEFAULT_CONTROL_MODE;

uint32_t static_motor_power_supply_voltage_mv_core1 = DEFAULT_STATIC_MOTOR_POWER_SUPPLY_VOLTAGE_MV;

uint8_t enable_simulated_servo_core1 = 0;

uint32_t simulated_servo_update_interval_us_core1_32bit = DEFAULT_SIMULATED_SERVO_UPDATE_INTERVAL_US;
uint64_t simulated_servo_update_interval_us_core1_64bit = DEFAULT_SIMULATED_SERVO_UPDATE_INTERVAL_US;
float simulated_servo_update_interval_s_core1 = DEFAULT_SIMULATED_SERVO_UPDATE_INTERVAL_US/1000000.0;

uint32_t simulated_servo_voltage_mv_core1 = DEFAULT_SIMULATED_SERVO_VOLTAGE_MV;
float simulated_servo_torque_constant_Nm_per_amp_core1 = DEFAULT_SIMULATED_SERVO_TORQUE_CONSTANT_NM_PER_AMP;
float simulated_servo_winding_resistance_ohms_core1 = DEFAULT_SIMULATED_SERVO_WINDING_RESISTANCE_OHMS;
uint32_t simulated_servo_counts_per_revolution_core1 = DEFAULT_SIMULATED_SERVO_COUNTS_PER_REVOLUTION;
float simulated_servo_inertia_kg_m2_core1 = DEFAULT_SIMULATED_SERVO_INERTIA_KG_M2;
float simulated_servo_damping_drag_coefficient_Nm_sec_per_rad_core1 = DEFAULT_SIMULATED_SERVO_DAMPING_DRAG_COEFFICIENT_NM_SEC_PER_RAD;
float simulated_servo_kinetic_friction_torque_Nm_core1 = DEFAULT_SIMULATED_SERVO_KINETIC_FRICTION_TORQUE_NM;

uint32_t proportional_gain_core1 = DEFAULT_PROPORTIONAL_GAIN;
uint32_t derivative_gain_core1 = DEFAULT_DERIVATIVE_GAIN;
uint32_t integral_gain_core1 = DEFAULT_INTEGRAL_GAIN;

uint32_t servo_update_interval_us_core1_32bit = DEFAULT_SERVO_UPDATE_INTERVAL_US;
uint64_t servo_update_interval_us_core1_64bit = DEFAULT_SERVO_UPDATE_INTERVAL_US;

uint32_t desired_state_update_interval_multiplier_core1 = DEFAULT_DESIRED_STATE_UPDATE_INTERVAL_MULTIPLIER;

uint8_t telemetry_reporting_mode_core1 = DEFAULT_TELEMETRY_REPORTING_MODE;
uint32_t telemetry_reporting_interval_multiplier_core1 = DEFAULT_TELEMETRY_REPORTING_MULTIPLIER;

uint8_t LED_mode_core1 = DEFAULT_LED_MODE;

float simulated_quadrature_counter = 0;
float simulated_quadrature_counter_dot = 0;

alarm_pool_t* core1_alarm_pool;
alarm_id_t servo_loop_interrupt;
alarm_id_t simulated_servo_interrupt;

int32_t commanded_control_voltage_mv;

uint slice_num;
uint channel_num_F;
uint channel_num_R;


/**
 * This function instantiates default values for all Core1 Exclusive RAM Variables. Shared variables are instantiated by Core0.
*/
void initialize_RAM_variables_core1(){   

    core1_alarm_pool = alarm_pool_create(1, 16);

    //Configure Motor Shield
    gpio_init(MOTOR_CONTROL_PIN_1);
    gpio_set_dir(MOTOR_CONTROL_PIN_1, GPIO_OUT);
    gpio_init(MOTOR_CONTROL_PIN_2);
    gpio_set_dir(MOTOR_CONTROL_PIN_2, GPIO_OUT);
    gpio_put(MOTOR_CONTROL_PIN_1, 0);
    gpio_put(MOTOR_CONTROL_PIN_2, 0);

    
    gpio_set_function(MOTOR_CONTROL_PIN_1, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(MOTOR_CONTROL_PIN_1);
    channel_num_R = pwm_gpio_to_channel(MOTOR_CONTROL_PIN_1);
    pwm_set_chan_level(slice_num, channel_num_R, 0);

    gpio_set_function(MOTOR_CONTROL_PIN_2, GPIO_FUNC_PWM);
    channel_num_F = pwm_gpio_to_channel(MOTOR_CONTROL_PIN_2);
    pwm_set_chan_level(slice_num, channel_num_F, 0);

    pwm_set_wrap(slice_num, 1250); //100kHz
    pwm_set_enabled(slice_num, true);
}