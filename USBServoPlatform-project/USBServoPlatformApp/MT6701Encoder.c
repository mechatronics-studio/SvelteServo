#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "MT6701Encoder.h"
#include "macro_header.h"
#include "hardware/pio.h"
#include "quadrature_encoder.pio.h"

/* REGISTER MAP & EEPROM PROGRAMMING
+--------------+-----------+-----------------+-----------------+-----------------+------------+------------+-----------+-----------+
| Reg. Address | bit7      | bit6            | bit5            | bit4            | bit3       | bit2       | bit1      | bit0      |
+--------------+-----------+-----------------+-----------------+-----------------+------------+------------+-----------+-----------+
| 37           | UVW_MUX   | XXXXXXXXXXXXXXX | XXXXXXXXXXXXXXX | XXXXXXXXXXXXXXX | XXXXXXXXXX | XXXXXXXXXX | XXXXXXXXX | XXXXXXXXX |
+--------------+-----------+-----------------+-----------------+-----------------+------------+------------+-----------+-----------+
| 41           | XXXXXXXXX | ABZ_MUX         | XXXXXXXXXXXXXXX | XXXXXXXXXXXXXXX | XXXXXXXXXX | XXXXXXXXXX | DIR       | XXXXXXXXX |
+--------------+-----------+-----------------+-----------------+-----------------+------------+------------+-----------+-----------+
| 48           | UVW_RES_3 | UVW_RES_2       | UVW_RES_1       | UVW_RES_0       | XXXXXXXXXX | XXXXXXXXXX | ABZ_RES_9 | ABZ_RES_8 |
+--------------+-----------+-----------------+-----------------+-----------------+------------+------------+-----------+-----------+
| 49           | ABZ_RES_7 | ABZ_RES_6       | ABZ_RES_5       | ABZ_RES_4       | ABZ_RES_3  | ABZ_RES_2  | ABZ_RES_1 | ABZ_RES_0 |
+--------------+-----------+-----------------+-----------------+-----------------+------------+------------+-----------+-----------+
| 50           | HYST_2    | Z_PULSE_WIDTH_2 | Z_PULSE_WIDTH_1 | Z_PULSE_WIDTH_0 | ZERO_11    | ZERO_10    | ZERO_9    | ZERO_8    |
+--------------+-----------+-----------------+-----------------+-----------------+------------+------------+-----------+-----------+
| 51           | ZERO_7    | ZERO_6          | ZERO_5          | ZERO_4          | ZERO_3     | ZERO_2     | ZERO_1    | ZERO_0    |
+--------------+-----------+-----------------+-----------------+-----------------+------------+------------+-----------+-----------+
| 52           | HYST_1    | HYST_0          | XXXXXXXXXXXXXXX | XXXXXXXXXXXXXXX | XXXXXXXXXX | XXXXXXXXXX | XXXXXXXXX | XXXXXXXXX |
+--------------+-----------+-----------------+-----------------+-----------------+------------+------------+-----------+-----------+
| 56           | PWM_FREQ  | PWM_POL         | OUT_MODE        | XXXXXXXXXXXXXXX | XXXXXXXXXX | XXXXXXXXXX | XXXXXXXXX | XXXXXXXXX |
+--------------+-----------+-----------------+-----------------+-----------------+------------+------------+-----------+-----------+
| 62           | A_STOP_11 | A_STOP_10       | A_STOP_9        | A_STOP_8        | A_START_11 | A_START_10 | A_START_9 | A_START_8 |
+--------------+-----------+-----------------+-----------------+-----------------+------------+------------+-----------+-----------+
| 63           | A_START_7 | A_START_6       | A_START_5       | A_START_4       | A_START_3  | A_START_2  | A_START_1 | A_START_0 |
+--------------+-----------+-----------------+-----------------+-----------------+------------+------------+-----------+-----------+
| 64           | A_STOP_7  | A_STOP_6        | A_STOP_5        | A_STOP_4        | A_STOP_3   | A_STOP_2   | A_STOP_1  | A_STOP_0  |
+--------------+-----------+-----------------+-----------------+-----------------+------------+------------+-----------+-----------+
UVW_MUX- UVW OUTPUT TYPE [QFN PACKAGE ONLY] 0:UVW 1:-A-B-Z
ABZ_MUX- ABZ OUTPUT TYPE 0:ABZ 1:UVW
DIR- ROTATION DIRECTION 0:CCW 1:CW
UVW_RES- UVW OUTPUT RESOLUTION, Between 1 and 16, STORED AS HEXADECIMAL EQUAL TO THE RESOLUTION -1 (ex. 0x1 = 2)
ABZ_RES- ABZ OUTPUT RESOLUTION (Pulses Per Revolution), Between 1 and 1024, STORED AS HEXADECIMAL EQUAL TO THE RESOLUTION -1 (ex. 0x1 = 2)
HYST- HYSTERESIS FILTER PARAMETER (0x0:1 0x1:2 0x2:4 0x3:8 0x4:0 0x5:.25 0x6:0.5 0x7:1)
Z_PULSE_WIDTH- WIDTH OF THE Z PULSE IN LEAST SIGNIFICANT BITS OR DEGREES (0x0:1 0x1:2 0x3:4 0x3:8 0x4:12 0x5:16 0x6:180degrees 0x7:1)
ZERO- ZERO DEGREE POSITION, GIVEN BY 360deg/4096*VALUE, WHERE VALUE IS BETWEEN 0 AND 4095
PWM_FREQ- PWM FRAME FREQUENCY (0x0:994.4hz 0x1:497.2hz)
PWM_POL- WHETHER PWM HIGH OR LOW IS DATA (0x0:HIGH 0x1:LOW)
OUT_MODE- DETERMINES THE TYPE OF OUTPUT FROM THE 'OUT' PIN (0x0:ANALOG 0x1:PWM_OUTPUT)
A_START- DETERMINES THE START ANGLE OF THE ANALOG OUTPUT, GIVEN BY 360deg/4096*VALUE, WHERE VALUE IS BETWEEN 0 AND 4095
A_STOP- DETERMINES THE STOP ANGLE OF THE ANALOG OUTPUT, GIVEN BY 360deg/4096*VALUE, WHERE VALUE IS BETWEEN 0 AND 4095
*/

const uint8_t abz_mux_register = 41;
const uint8_t abz_mux_bit = 6;

const uint8_t dir_register = 41;
const uint8_t dir_bit = 1;

const uint8_t abz_resolution_bits_9to8_register = 48;
const uint8_t abz_resolution_bits_9to8_least_significant_bit = 0;
const uint8_t abz_resolution_bits_9to8_most_significant_bit = 1;

const uint8_t abz_resolution_bits_7to0_register = 49;
const uint8_t abz_resolution_bits_7to0_least_significant_bit = 0;
const uint8_t abz_resolution_bits_7to0_most_significant_bit = 7;

const uint8_t hyst_2_bit_register = 50;
const uint8_t hyst_2_bit = 7;

const uint8_t hyst_bits_1to0_register = 52;
const uint8_t hyst_bits_1to0_least_significant_bit = 6;
const uint8_t hyst_bits_1to0_most_significant_bit = 7;

const uint8_t z_pulse_width_register = 50;
const uint8_t z_pulse_width_least_significant_bit = 4;
const uint8_t z_pulse_width_most_significant_bit = 6;

const uint8_t zero_position_bits_11to8_register = 50;
const uint8_t zero_position_bits_11to8_register_least_significant_bit = 0;
const uint8_t zero_position_bits_11to8_register_most_significant_bit = 3;

const uint8_t zero_position_bits_7to0_register = 51;
const uint8_t zero_position_bits_7to0_register_least_significant_bit = 0;
const uint8_t zero_position_bits_7to0_register_most_significant_bit = 7;

const uint8_t out_mode_register = 56;
const uint8_t out_mode_bit = 5;

const uint8_t a_stop_bits_11to8_register = 62;
const uint8_t a_stop_bits_11to8_register_least_significant_bit = 4;
const uint8_t a_stop_bits_11to8_register_most_significant_bit = 7;

const uint8_t a_start_bits_11to8_register = 62;
const uint8_t a_start_bits_11to8_register_least_significant_bit = 0;
const uint8_t a_start_bits_11to8_register_most_significant_bit = 3;

const uint8_t a_start_bits_7to0_register = 63;
const uint8_t a_start_bits_7to0_register_least_significant_bit = 0;
const uint8_t a_start_bits_7to0_register_most_significant_bit = 7;

const uint8_t a_stop_bits_7to0_register = 64;
const uint8_t a_stop_bits_7to0_register_least_significant_bit = 0;
const uint8_t a_stop_bits_7to0_register_most_significant_bit = 7;

const uint8_t absolute_position_bits_13to6_register = 3;

const uint8_t absolute_position_bits_5to0_register = 4;
const uint8_t absolute_position_bits_5to0_least_significant_bit = 2;
const uint8_t absolute_position_bits_5to0_most_significant_bit = 7;

const uint8_t mt6701_i2c_address = 6;

uint8_t i2c_buffer;

uint8_t generate_bit_mask(uint8_t most_signifcant_bit, uint8_t least_significant_bit) {
    uint8_t mask = 0;

    for (uint8_t i = least_significant_bit; i <= most_signifcant_bit; i++) {
        mask = mask | (1 << i);
    }

    return mask;
}

int32_t read_abz_mux_bit(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&abz_mux_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t abz_mux_bit_result = i2c_buffer && 0b00000001 << abz_mux_bit;

    return abz_mux_bit_result;
}

const char* decode_abz_mux_value(int32_t bit_value){
    switch(bit_value){
        case 0:
            return "ABZ";
        case 1:
            return "UVW";
        default:
            return "INVALID ABZ MUX BIT VALUE";
    }
}

int32_t read_dir_bit(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&dir_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t dir_bit_result = i2c_buffer && 0b00000001 << dir_bit;

    return dir_bit_result;
}

const char* decode_dir_value(int32_t bit_value){
    switch(bit_value){
        case 0:
            return "CCW";
        case 1:
            return "CW";
        default:
            return "INVALID DIRECTION BIT VALUE";
    }
}

int32_t read_abz_resolution_value(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&abz_resolution_bits_9to8_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t abz_resolution_bits_9to8_result = i2c_buffer && generate_bit_mask(abz_resolution_bits_9to8_most_significant_bit,abz_resolution_bits_9to8_least_significant_bit);

    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&abz_resolution_bits_7to0_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t abz_resolution_bits_7to0_result = i2c_buffer;

    int32_t abz_resolution = abz_resolution_bits_9to8_most_significant_bit << 8 || abz_resolution_bits_7to0_result;


    return abz_resolution;
}

char string_buffer[10];

//This function converts the abz_resolution value read from EEPROM to the actual ABZ resolution. If the abz_resolution value is invalid, -1 is returned.
int32_t decode_abz_resolution_value(int32_t field_value){
    if(field_value <= 1023){
        return field_value+1;
    }
    else{
        return -1;
    }

}

int8_t read_hyst_value(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&hyst_2_bit_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t hyst_2_bit_result = i2c_buffer && 0b00000001 << hyst_2_bit;

    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&hyst_bits_1to0_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t hyst_bits_1to0_result = i2c_buffer && generate_bit_mask(hyst_bits_1to0_most_significant_bit,hyst_bits_1to0_least_significant_bit);


    int32_t hyst = hyst_2_bit_result << 3 && hyst_bits_1to0_result;

    return hyst;
}

//This function converts the hyst field value read from EEPROM to the actual hyst setting value. If the hyst field value is invalid, -1 is returned.
float decode_hyst_value(int32_t field_value){
    switch(field_value){
        case 0:
            return 1;
        case 1:
            return 2;
        case 2:
            return 4;
        case 3:
            return 8;
        case 4:
            return 0;
        case 5:
            return 0.25;
        case 6:
            return 0.5;
        case 7:
            return 1;  
        default:
            return -1;
    }
}

int32_t read_z_pulse_width_value(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&z_pulse_width_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t z_pulse_width_result = i2c_buffer && generate_bit_mask(z_pulse_width_most_significant_bit,z_pulse_width_least_significant_bit);

    return z_pulse_width_result;
}

//Convert the z_pulse_width field value read from EEPROM to the actual z_pulse_width setting value. If the z_pulse_width field value is invalid, an invalid error is returned.
const char* decode_z_pulse_width_value(int32_t field_value){
    switch(field_value){
        case 0:
            return "1 LSB";
        case 1:
            return "2 LSB";
        case 2:
            return "4 LSB";
        case 3:
            return "8 LSB";
        case 4:
            return "12 LSB";
        case 5:
            return "16 LSB";
        case 6:
            return "180 DEG";
        case 7:
            return "1 LSB";  
        default:
            return "INVALID Z_PULSE_WIDTH FIELD VALUE";
    }
}

uint32_t read_zero_position_value(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&zero_position_bits_11to8_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t zero_position_bits_11to8_result = i2c_buffer && generate_bit_mask(zero_position_bits_11to8_register_most_significant_bit,zero_position_bits_11to8_register_least_significant_bit);

    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&zero_position_bits_7to0_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t zero_position_bits_7to0_result = i2c_buffer;

    int32_t zero_position = zero_position_bits_11to8_result << 8 || zero_position_bits_7to0_result;


    return zero_position;
}

//Convert the zero_position field value read from EEPROM to the actual zero_position setting value in degrees. If the zero_position field value is invalid, -1 is returned.
float decode_zero_position_value(int32_t field_value){
    if(field_value <= 4095){
        return (float)field_value*360/4096;
    }
    else{
        return -1;
    }
}


uint32_t read_out_mode_value(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&out_mode_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t dir_bit_result = i2c_buffer && 0b00000001 << out_mode_bit;

    return dir_bit_result;
}

const char* decode_out_mode_value(int32_t bit_value){
    switch(bit_value){
        case 0:
            return "ANALOG OUTPUT";
        case 1:
            return "PWM OUTPUT";
    }
}

uint32_t read_a_stop_position_value(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&a_stop_bits_11to8_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t a_stop_bits_11to8_result = i2c_buffer && generate_bit_mask(a_stop_bits_11to8_register_most_significant_bit,a_stop_bits_11to8_register_least_significant_bit);

    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&a_stop_bits_7to0_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t a_stop_bits_7to0_result = i2c_buffer;

    int32_t a_stop = a_stop_bits_11to8_result << 8 || a_stop_bits_7to0_result;


    return a_stop;
}

//Convert the zero_position field value read from EEPROM to the actual zero_position setting value in degrees. If the zero_position field value is invalid, -1 is returned.
float decode_a_stop_value(int32_t field_value){
    if(field_value <= 4095){
        return (float)field_value*360/4096;
    }
    else{
        return -1;
    }
}

uint32_t read_a_start_position_value(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&a_start_bits_11to8_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t a_start_bits_11to8_result = i2c_buffer && generate_bit_mask(a_start_bits_11to8_register_most_significant_bit,a_start_bits_11to8_register_least_significant_bit);

    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&a_start_bits_7to0_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t a_start_bits_7to0_result = i2c_buffer;

    int32_t a_start = a_start_bits_11to8_result << 8 || a_start_bits_7to0_result;


    return a_start;
}

//Convert the zero_position field value read from EEPROM to the actual zero_position setting value in degrees. If the zero_position field value is invalid, -1 is returned.
float decode_a_start_value(int32_t field_value){
    if(field_value <= 4095){
        return (float)field_value*360/4096;
    }
    else{
        return -1;
    }
}

int32_t read_current_absolute_position(){
    int32_t status = 0;/*
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&absolute_position_bits_13to6_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t absolute_position_bits_13to6_result = i2c_buffer;

    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&absolute_position_bits_5to0_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }
    
    int32_t absolute_position_bits_5to0_result = i2c_buffer && generate_bit_mask(absolute_position_bits_5to0_most_significant_bit,absolute_position_bits_5to0_least_significant_bit);
*/
    int32_t absolute_position = 4;//(absolute_position_bits_13to6_result << 6) || absolute_position_bits_5to0_result;

    return absolute_position;
}

float decode_absolute_position(int32_t field_value){
    return (float)field_value*360/16384;
}

int32_t read_and_report_via_usb_MT6701_eeprom_values(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&absolute_position_bits_13to6_register,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }



}

/*Activates i2C pins and checks for and return true if a valid position response is recieved from MT6701 encoder.*/
void begin_i2c_with_MT6701(){
    gpio_set_function(MT6701_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MT6701_I2C_SCL_PIN, GPIO_FUNC_I2C);
    if(read_current_absolute_position() == PICO_ERROR_GENERIC){
        printf("CRITICAL ERROR: MT6701 ENCODER NOT RESPONSIVE\n");
    }    
}

void end_i2c_with_MT6701(){
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_NULL);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_NULL);
}

PIO pio = pio0;
const uint sm = 0;
const uint MT6701_channel_a_pin = MT6701_CHANNEL_A_PIN;

void initialize_MT6701(){
    gpio_pull_up(MT6701_ENCODER_MODE_PIN);
    gpio_set_dir(MT6701_ENCODER_MODE_PIN,true);
    gpio_put(MT6701_ENCODER_MODE_PIN,true);

    gpio_pull_up(MT6701_CHANNEL_A_PIN);
    gpio_pull_up(MT6701_CHANNEL_B_PIN);

    gpio_pull_up(MT6701_I2C_SDA_PIN);
    gpio_pull_up(MT6701_I2C_SCL_PIN);
    i2c_init(i2c_default,100000);

    begin_i2c_with_MT6701();

    int32_t dir = read_dir_bit();
    printf("ENCODER DIR BIT: ");
    printf(decode_dir_value(dir));
    printf("\n");

    end_i2c_with_MT6701();

    //uint offset = pio_add_program(pio, &quadrature_encoder_program);
    //quadrature_encoder_program_init(pio, sm, offset, MT6701_channel_a_pin, 0);
}

void print_current_absolute_position(){
    //begin_i2c_with_MT6701();
    int32_t current_position = read_current_absolute_position();
    float decoded_absolute_position = decode_absolute_position(current_position);
    printf("CURRENT POSITION: %f\n",decoded_absolute_position);
    //end_i2c_with_MT6701();
}




