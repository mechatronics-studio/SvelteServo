#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "MT6701Encoder.h"
#include "macro_header.h"
#include "hardware/pio.h"
#include "quadrature_encoder.pio.h"

#define INVALID_VALUE_TO_ENCODE -1

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

const uint8_t abz_mux_register_address = 41;
const uint8_t abz_mux_bit = 6;

const uint8_t direction_register_address = 41;
const uint8_t direction_bit = 1;

const uint8_t abz_resolution_bits_9to8_register_address = 48;
const uint8_t abz_resolution_bits_9to8_least_significant_bit = 0;
const uint8_t abz_resolution_bits_9to8_most_significant_bit = 1;
const uint8_t abz_resolution_bits_9to8_size = abz_resolution_bits_9to8_most_significant_bit-abz_resolution_bits_9to8_least_significant_bit+1;

const uint8_t abz_resolution_bits_7to0_register_address = 49;
const uint8_t abz_resolution_bits_7to0_least_significant_bit = 0;
const uint8_t abz_resolution_bits_7to0_most_significant_bit = 7;
const uint8_t abz_resolution_bits_7to0_size = abz_resolution_bits_7to0_most_significant_bit-abz_resolution_bits_7to0_least_significant_bit+1;

const uint8_t hyst_2_bit_register_address = 50;
const uint8_t hyst_2_bit = 7;

const uint8_t hyst_bits_1to0_register_address = 52;
const uint8_t hyst_bits_1to0_least_significant_bit = 6;
const uint8_t hyst_bits_1to0_most_significant_bit = 7;
const uint8_t hyst_bits_1to0_size = hyst_bits_1to0_most_significant_bit-hyst_bits_1to0_least_significant_bit+1;

const uint8_t z_pulse_width_register_address = 50;
const uint8_t z_pulse_width_least_significant_bit = 4;
const uint8_t z_pulse_width_most_significant_bit = 6;
const uint8_t z_pulse_width_size = z_pulse_width_most_significant_bit-z_pulse_width_least_significant_bit+1;

const uint8_t zero_position_bits_11to8_register_address = 50;
const uint8_t zero_position_bits_11to8_least_significant_bit = 0;
const uint8_t zero_position_bits_11to8_most_significant_bit = 3;
const uint8_t zero_position_bits_11to8_size = zero_position_bits_11to8_most_significant_bit-zero_position_bits_11to8_least_significant_bit+1;

const uint8_t zero_position_bits_7to0_register_address = 51;
const uint8_t zero_position_bits_7to0_least_significant_bit = 0;
const uint8_t zero_position_bits_7to0_most_significant_bit = 7;
const uint8_t zero_position_bits_7to0_size = zero_position_bits_7to0_most_significant_bit-zero_position_bits_7to0_least_significant_bit+1;

const uint8_t out_mode_register_address = 56;
const uint8_t out_mode_bit = 5;

const uint8_t a_start_bits_11to8_register_address = 62;
const uint8_t a_start_bits_11to8_least_significant_bit = 0;
const uint8_t a_start_bits_11to8_most_significant_bit = 3;
const uint8_t a_start_bits_11to8_size = a_start_bits_11to8_most_significant_bit-a_start_bits_11to8_least_significant_bit+1;

const uint8_t a_start_bits_7to0_register_address = 63;
const uint8_t a_start_bits_7to0_least_significant_bit = 0;
const uint8_t a_start_bits_7to0_most_significant_bit = 7;
const uint8_t a_start_bits_7to0_size = a_start_bits_7to0_most_significant_bit-a_start_bits_7to0_least_significant_bit+1;

const uint8_t a_stop_bits_11to8_register_address = 62;
const uint8_t a_stop_bits_11to8_least_significant_bit = 4;
const uint8_t a_stop_bits_11to8_most_significant_bit = 7;
const uint8_t a_stop_bits_11to8_size = a_stop_bits_11to8_most_significant_bit-a_stop_bits_11to8_least_significant_bit+1;

const uint8_t a_stop_bits_7to0_register_address = 64;
const uint8_t a_stop_bits_7to0_least_significant_bit = 0;
const uint8_t a_stop_bits_7to0_most_significant_bit = 7;
const uint8_t a_stop_bits_7to0_size = a_stop_bits_7to0_most_significant_bit-a_stop_bits_7to0_least_significant_bit+1;

const uint8_t absolute_position_bits_13to6_register_address = 3;
const uint8_t absolute_position_bits_13to6_least_significant_bit = 0;
const uint8_t absolute_position_bits_13to6_most_significant_bit = 7;
const uint8_t absolute_position_bits_13to6_size = absolute_position_bits_13to6_most_significant_bit-absolute_position_bits_13to6_least_significant_bit+1;

const uint8_t absolute_position_bits_5to0_register_address = 4;
const uint8_t absolute_position_bits_5to0_least_significant_bit = 2;
const uint8_t absolute_position_bits_5to0_most_significant_bit = 7;
const uint8_t absolute_position_bits_5to0_size = absolute_position_bits_5to0_most_significant_bit-absolute_position_bits_5to0_least_significant_bit+1;

const uint8_t mt6701_i2c_address = 6;

const uint8_t programming_key_step1[] = {0x09,0xB3}; 
const uint8_t programming_command_step2[] = {0x0A,0x05};

uint8_t i2c_buffer;

uint8_t generate_bit_mask(uint8_t most_signifcant_bit, uint8_t least_significant_bit) {
    uint8_t mask = 0;

    for (uint8_t i = least_significant_bit; i <= most_signifcant_bit; i++) {
        mask = mask | (1 << i);
    }

    return mask;
}

uint8_t extract_bitfield(uint8_t register_value, uint8_t most_significant_bit, uint8_t least_significant_bit){
return (register_value & generate_bit_mask(most_significant_bit,least_significant_bit)) >> least_significant_bit;
}

uint8_t extract_bit(uint8_t register_value, uint8_t bit_position){
return (register_value & (0b00000001 << bit_position)) >> bit_position;
}

int32_t merge_two_bitfields(uint8_t upper_register_value, uint8_t lower_register_value, uint8_t size_of_lower_register){
    return (upper_register_value << size_of_lower_register) | lower_register_value;
}

uint8_t merge_bit_onto_register(uint8_t register_value, uint8_t bit_value, uint8_t bit_position){
    return (register_value & (0b11111111 ^ (0b00000001 << bit_position))) | (bit_value << bit_position);
}

uint8_t merge_bitfield_onto_register(uint8_t register_value, uint8_t bitfield_value, uint8_t most_significant_bit, uint8_t least_significant_bit){
    // Calculate the bit mask for the given range
    uint8_t mask = (1 << (most_significant_bit - least_significant_bit + 1)) - 1;
    mask <<= least_significant_bit;

    // Clear the bits in the register for the given range
    register_value &= ~mask;

    // Shift the bitfield value to the correct position and merge it onto the register
    bitfield_value <<= least_significant_bit;
    register_value |= bitfield_value;

    // Return the updated register value
    return register_value;
}


    //uint8_t bitfield_mask = generate_bit_mask(most_significant_bit,least_significant_bit);
    //return ((register_value & bitfield_mask) & (bitfield_value << least_significant_bit)) | register_value;
void write_eeprom_programming_sequence(){
    i2c_write_blocking(i2c_default,mt6701_i2c_address,programming_key_step1,2,false);
    i2c_write_blocking(i2c_default,mt6701_i2c_address,programming_command_step2,2,false);
    sleep_ms(1000);
}

void modify_register_bit_and_write_register_to_eeprom(uint8_t register_address, uint8_t bit_value, uint8_t bit_position){
    i2c_write_blocking(i2c_default,mt6701_i2c_address,&register_address,1,true);
    i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);

    uint8_t register_value = i2c_buffer;

    if(extract_bit(register_value,bit_position) != bit_value){
        uint8_t new_register_value = merge_bit_onto_register(register_value,bit_value,bit_position);
        uint8_t i2c_data_to_write[] = {register_address,new_register_value};
        i2c_write_blocking(i2c_default,mt6701_i2c_address,i2c_data_to_write,2,false);
    }
}

void modify_register_bitfield_and_write_register_to_eeprom(uint8_t register_address, uint8_t bitfield_value, uint8_t most_significant_bit, uint8_t least_significant_bit){
    i2c_write_blocking(i2c_default,mt6701_i2c_address,&register_address,1,true);
    i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);

    uint8_t register_value = i2c_buffer;

    if(extract_bitfield(register_value,most_significant_bit,least_significant_bit) != bitfield_value){
        uint8_t new_register_value = merge_bitfield_onto_register(register_value,bitfield_value,most_significant_bit,least_significant_bit);
        uint8_t i2c_data_to_write[] = {register_address,new_register_value};
        i2c_write_blocking(i2c_default,mt6701_i2c_address,i2c_data_to_write,2,false);
    }
}

int32_t read_abz_mux_bit(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&abz_mux_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t abz_mux_bit_result = extract_bit(i2c_buffer,abz_mux_bit);

    return abz_mux_bit_result;
}

const char* decode_abz_mux_value(int32_t bit_value){
    switch(bit_value){
        case 0:
            return ABZ_KEY;
        case 1:
            return UVW_KEY;
        default:
            return "INVALID ABZ MUX BIT VALUE";
    }
}

int8_t encode_abz_mux_key(const char* key){
    if(key == ABZ_KEY){
        return 0;
    }
    else if(key == UVW_KEY){
        return 1;
    }
    else{
        return 0;
    }
}

//Reads eeprom and checks if abz_mux key matches abz_mux_key_to_enforce. If it does not, abz_mux_key_to_enforce is converted to the corresponding value 
//and is written to the device (but not programmed).
bool enforce_abz_mux_by_key(const char* abz_mux_key_to_enforce){
        int32_t abz_mux_value_in_eeprom = read_abz_mux_bit();
        if(abz_mux_value_in_eeprom == PICO_ERROR_GENERIC){
            printf("CRITICAL ERROR: MT6701 ENCODER NOT RESPONSIVE\n");
        }

        uint8_t abz_mux_value_to_enforce = encode_abz_mux_key(abz_mux_key_to_enforce);

        if(decode_abz_mux_value(abz_mux_value_in_eeprom) != abz_mux_key_to_enforce){
            printf("ABZ_MUX EEPROM VALUE/KEY DOES NOT MATCH ENFORCED VALUE. WRITING ENFORCED VALUE/KEY: %d|%s.\n", abz_mux_value_to_enforce, abz_mux_key_to_enforce);

            modify_register_bit_and_write_register_to_eeprom(abz_mux_register_address,abz_mux_value_to_enforce,abz_mux_bit);

            printf("ABZ_MUX EEPROM VALUE/KEY WRITTEN. NOT YET PROGRAMMED.\n");
            return true;
        }
        else{
            return false;
        }
}

int32_t read_direction_bit(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&direction_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t direction_bit_result = extract_bit(i2c_buffer,direction_bit);

    return direction_bit_result;
}

const char* decode_direction_value(int32_t bit_value){
    switch(bit_value){
        case 0:
            return CCW_KEY;
        case 1:
            return CW_KEY;
        default:
            return "INVALID DIRECTION BIT VALUE";
    }
}

int8_t encode_direction_key(const char* key){
    if(key == CCW_KEY){
        return 0;
    }
    else if(key == CW_KEY){
        return 1;
    }
    else{
        return 0;
    }
}

//Reads eeprom and checks if direction_key matches direction_key_to_enforce. If it does not, direction_key_to_enforce is converted to the corresponding value 
//and is written to the device (but not programmed).
bool enforce_direction_by_key(const char* direction_key_to_enforce){
        int32_t direction_value_in_eeprom = read_direction_bit();
        if(direction_value_in_eeprom == PICO_ERROR_GENERIC){
            printf("CRITICAL ERROR: MT6701 ENCODER NOT RESPONSIVE\n");
        }

        uint8_t direction_value_to_enforce = encode_direction_key(direction_key_to_enforce);

        if(decode_direction_value(direction_value_in_eeprom) != direction_key_to_enforce){
            printf("DIRECTION EEPROM VALUE/KEY DOES NOT MATCH ENFORCED VALUE. WRITING ENFORCED VALUE/KEY: %d|%s.\n", direction_value_to_enforce, direction_key_to_enforce);

            modify_register_bit_and_write_register_to_eeprom(direction_register_address,direction_value_to_enforce,direction_bit);

            printf("DIRECTION EEPROM VALUE/KEY WRITTEN. NOT YET PROGRAMMED.\n");
            return true;
        }
        else{
            return false;
        }
}

int32_t read_abz_resolution_value(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&abz_resolution_bits_9to8_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    uint8_t abz_resolution_bits_9to8_result = extract_bitfield(i2c_buffer,abz_resolution_bits_9to8_most_significant_bit,abz_resolution_bits_9to8_least_significant_bit);
    
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&abz_resolution_bits_7to0_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    uint8_t abz_resolution_bits_7to0_result = extract_bitfield(i2c_buffer,abz_resolution_bits_7to0_most_significant_bit,abz_resolution_bits_7to0_least_significant_bit);

    int32_t abz_resolution = merge_two_bitfields(abz_resolution_bits_9to8_result,abz_resolution_bits_7to0_result,abz_resolution_bits_7to0_size);

    return abz_resolution;
}

//This function converts the abz_resolution value read from EEPROM to the actual ABZ resolution. If the abz_resolution value is invalid, -1 is returned.
int32_t decode_abz_resolution_value(int32_t field_value){
    if(field_value <= 1023){
        return field_value+1;
    }
    else{
        return -1;
    }

}

//This function converts an abz_resolution key to the cooresponding eeprom value. If the abz_resolution value is invalid, -1 is returned.
int32_t encode_abz_resolution_key(int32_t key){
    if(key <= 1024 && key > 0){
        return key-1;
    }
    else{
        return -1;
    }

}

//Reads eeprom and checks if direction_key matches direction_key_to_enforce. If it does not, direction_key_to_enforce is converted to the corresponding value 
//and is written to the device (but not programmed).
bool enforce_abz_resolution_by_key(int32_t abz_resolution_key_to_enforce){
        int32_t abz_resolution_value_in_eeprom = read_abz_resolution_value();
        if(abz_resolution_value_in_eeprom == PICO_ERROR_GENERIC){
            printf("CRITICAL ERROR: MT6701 ENCODER NOT RESPONSIVE\n");
        }

        int32_t abz_resolution_value_to_enforce = encode_abz_resolution_key(abz_resolution_key_to_enforce);
        if(decode_abz_resolution_value(abz_resolution_value_in_eeprom) != abz_resolution_key_to_enforce){
            printf("ABZ_RESOLUTION EEPROM VALUE/KEY DOES NOT MATCH ENFORCED VALUE. WRITING ENFORCED VALUE/KEY: %d|%d.\n", abz_resolution_value_to_enforce, abz_resolution_key_to_enforce);

            uint8_t abz_resolution_bits_7to0_to_enforce = abz_resolution_value_to_enforce & generate_bit_mask(abz_resolution_bits_7to0_most_significant_bit,abz_resolution_bits_7to0_least_significant_bit) ;
            uint8_t abz_resolution_bits_9to8_to_enforce = abz_resolution_value_to_enforce >> abz_resolution_bits_7to0_size;
            
            modify_register_bitfield_and_write_register_to_eeprom(abz_resolution_bits_7to0_register_address,abz_resolution_bits_7to0_to_enforce,abz_resolution_bits_7to0_most_significant_bit,abz_resolution_bits_7to0_least_significant_bit);
            modify_register_bitfield_and_write_register_to_eeprom(abz_resolution_bits_9to8_register_address,abz_resolution_bits_9to8_to_enforce,abz_resolution_bits_9to8_most_significant_bit,abz_resolution_bits_9to8_least_significant_bit);

            printf("ABZ_RESOLUTION EEPROM VALUE/KEY WRITTEN. NOT YET PROGRAMMED.\n");
            return true;
        }
        else{
            return false;
        }
}

int8_t read_hyst_value(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&hyst_2_bit_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    uint8_t hyst_2_bit_result = extract_bit(i2c_buffer,hyst_2_bit);

    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&hyst_bits_1to0_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    uint8_t hyst_bits_1to0_result = extract_bitfield(i2c_buffer,hyst_bits_1to0_most_significant_bit,hyst_bits_1to0_least_significant_bit);


    int32_t hyst = merge_two_bitfields(hyst_2_bit_result,hyst_bits_1to0_result,hyst_bits_1to0_size);

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
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&z_pulse_width_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t z_pulse_width_result = extract_bitfield(i2c_buffer,z_pulse_width_most_significant_bit,z_pulse_width_least_significant_bit);

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
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&zero_position_bits_11to8_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    uint8_t zero_position_bits_11to8_result = extract_bitfield(i2c_buffer,zero_position_bits_11to8_most_significant_bit,zero_position_bits_11to8_least_significant_bit);

    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&zero_position_bits_7to0_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    uint8_t zero_position_bits_7to0_result = extract_bitfield(i2c_buffer,zero_position_bits_7to0_most_significant_bit,zero_position_bits_7to0_least_significant_bit);

    int32_t zero_position = merge_two_bitfields(zero_position_bits_11to8_result,zero_position_bits_7to0_result,zero_position_bits_7to0_size);


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
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&out_mode_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    int32_t out_mode_bit_result = extract_bit(i2c_buffer,out_mode_bit);

    return out_mode_bit_result;
}

const char* decode_out_mode_value(int32_t bit_value){
    switch(bit_value){
        case 0:
            return ANALOG_KEY;
        case 1:
            return PWM_KEY;
        default:
            return "INVALID OUT_MODE BIT VALUE";;
    }
}

int32_t encode_out_mode_key(const char* key){
    if(key == ANALOG_KEY){
        return 0;
    }
    else if(key == PWM_KEY){
        return 1;
    }
    else{
        return -1;
    }
}

//Reads eeprom and checks if direction_key matches direction_key_to_enforce. If it does not, direction_key_to_enforce is converted to the corresponding value 
//and is written to the device (but not programmed).
bool enforce_out_mode_by_key(const char* out_mode_key_to_enforce){
        int32_t out_mode_value_in_eeprom = read_out_mode_value();
        if(out_mode_value_in_eeprom == PICO_ERROR_GENERIC){
            printf("CRITICAL ERROR: MT6701 ENCODER NOT RESPONSIVE\n");
        }

        uint8_t out_mode_value_to_enforce = encode_out_mode_key(out_mode_key_to_enforce);

        if(decode_out_mode_value(out_mode_value_in_eeprom) != out_mode_key_to_enforce){
            printf("OUT_MODE EEPROM VALUE/KEY DOES NOT MATCH ENFORCED VALUE. WRITING ENFORCED VALUE/KEY: %d|%s.\n", out_mode_value_to_enforce, out_mode_key_to_enforce);

            modify_register_bit_and_write_register_to_eeprom(out_mode_register_address,out_mode_value_to_enforce,out_mode_bit);

            printf("OUT_MODE EEPROM VALUE/KEY WRITTEN. NOT YET PROGRAMMED.\n");
            return true;
        }
        else{
            return false;
        }
}

uint32_t read_a_stop_position_value(){
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&a_stop_bits_11to8_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    uint8_t a_stop_bits_11to8_result = extract_bitfield(i2c_buffer,a_stop_bits_11to8_most_significant_bit,a_stop_bits_11to8_least_significant_bit);

    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&a_stop_bits_7to0_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    uint8_t a_stop_bits_7to0_result = extract_bitfield(i2c_buffer,a_stop_bits_7to0_most_significant_bit,a_stop_bits_7to0_least_significant_bit);

    int32_t a_stop = merge_two_bitfields(a_stop_bits_11to8_result,a_stop_bits_7to0_result,a_stop_bits_7to0_size);


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
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&a_start_bits_11to8_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    uint8_t a_start_bits_11to8_result = extract_bitfield(i2c_buffer,a_start_bits_11to8_most_significant_bit,a_start_bits_11to8_least_significant_bit);

    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&a_start_bits_7to0_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    uint8_t a_start_bits_7to0_result = extract_bitfield(i2c_buffer,a_start_bits_7to0_most_significant_bit,a_start_bits_7to0_least_significant_bit);

    int32_t a_start = merge_two_bitfields(a_start_bits_11to8_result,a_start_bits_7to0_result,a_start_bits_7to0_size);

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
    int32_t status = 0;
    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&absolute_position_bits_13to6_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }

    uint8_t absolute_position_bits_13to6_result = extract_bitfield(i2c_buffer,absolute_position_bits_13to6_most_significant_bit,absolute_position_bits_13to6_least_significant_bit);

    status = i2c_write_blocking(i2c_default,mt6701_i2c_address,&absolute_position_bits_5to0_register_address,1,true);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    } 

    status = i2c_read_blocking(i2c_default,mt6701_i2c_address,&i2c_buffer,1,false);
    if(status == PICO_ERROR_GENERIC){
        return PICO_ERROR_GENERIC;
    }
    
    int32_t absolute_position_bits_5to0_result = extract_bitfield(i2c_buffer,absolute_position_bits_5to0_most_significant_bit,absolute_position_bits_5to0_least_significant_bit);
    int32_t absolute_position = merge_two_bitfields(absolute_position_bits_13to6_result,absolute_position_bits_5to0_result,absolute_position_bits_5to0_size);

    return absolute_position;
}

float decode_absolute_position(int32_t field_value){
    return (float)field_value*360/16384;
}

int32_t read_and_report_via_usb_MT6701_eeprom_values(){
    int32_t abz_mux = read_abz_mux_bit();
    if(abz_mux == PICO_ERROR_GENERIC){
        printf("CRITICAL ERROR: MT6701 ENCODER NOT RESPONSIVE\n");
    }
    printf("MT6701 ABZ_MUX BIT: %s\n",decode_abz_mux_value(abz_mux));

    int32_t dir = read_direction_bit();
    if(dir == PICO_ERROR_GENERIC){
        printf("CRITICAL ERROR: MT6701 ENCODER NOT RESPONSIVE\n");
    }
    printf("MT6701 DIR BIT: %s\n",decode_direction_value(dir));
    

    int32_t abz_resolution = read_abz_resolution_value();
    if(abz_resolution == PICO_ERROR_GENERIC){
        printf("CRITICAL ERROR: MT6701 ENCODER NOT RESPONSIVE\n");
    }
    printf("MT6701 ABZ_RES: %ld\n",decode_abz_resolution_value(abz_resolution));

    int32_t hyst = read_hyst_value();
    if(hyst == PICO_ERROR_GENERIC){
        printf("CRITICAL ERROR: MT6701 ENCODER NOT RESPONSIVE\n");
    }
    printf("MT6701 HYST: %f\n",decode_hyst_value(hyst));

    int32_t z_pulse_width = read_z_pulse_width_value();
    if(z_pulse_width == PICO_ERROR_GENERIC){
        printf("CRITICAL ERROR: MT6701 ENCODER NOT RESPONSIVE\n");
    }
    printf("MT6701 Z_PULSE_WIDTH: %s\n",decode_z_pulse_width_value(z_pulse_width));

    int32_t zero_position = read_zero_position_value();
    if(zero_position == PICO_ERROR_GENERIC){
        printf("CRITICAL ERROR: MT6701 ENCODER NOT RESPONSIVE\n");
    }
    printf("MT6701 ZERO_POSITION: %f\n",decode_zero_position_value(zero_position));

    int32_t out_mode = read_out_mode_value();
    if(out_mode == PICO_ERROR_GENERIC){
        printf("CRITICAL ERROR: MT6701 ENCODER NOT RESPONSIVE\n");
    }
    printf("MT6701 OUT_MODE: %s\n",decode_out_mode_value(out_mode));

    int32_t analog_start_position = read_a_start_position_value();
    if(analog_start_position == PICO_ERROR_GENERIC){
        printf("CRITICAL ERROR: MT6701 ENCODER NOT RESPONSIVE\n");
    }
    printf("MT6701 ANALOG_START_POSITION: %f\n",decode_a_start_value(analog_start_position));

    int32_t analog_stop_position = read_a_stop_position_value();
    if(analog_stop_position == PICO_ERROR_GENERIC){
        printf("CRITICAL ERROR: MT6701 ENCODER NOT RESPONSIVE\n");
    }
    printf("MT6701 ANALOG_STOP_POSITION: %f\n",decode_a_stop_value(analog_stop_position));
    
}

/*Activates i2C pins and checks for and return true if a valid position response is recieved from MT6701 encoder.*/
void begin_i2c_with_MT6701(){
    gpio_set_function(MT6701_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MT6701_I2C_SCL_PIN, GPIO_FUNC_I2C);
    i2c_init(i2c_default,100000);
}

void end_i2c_with_MT6701(){
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_NULL);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_NULL);
}

PIO pio = pio0;
const uint sm = 0;

void initialize_MT6701(){
    gpio_init(MT6701_ENCODER_MODE_PIN);
    gpio_pull_up(MT6701_ENCODER_MODE_PIN);
    gpio_set_dir(MT6701_ENCODER_MODE_PIN,true);
    gpio_put(MT6701_ENCODER_MODE_PIN,true);

    gpio_pull_up(MT6701_CHANNEL_A_PIN);
    gpio_pull_up(MT6701_CHANNEL_B_PIN);

    gpio_pull_up(MT6701_I2C_SDA_PIN);
    gpio_pull_up(MT6701_I2C_SCL_PIN);

    begin_i2c_with_MT6701();
    
    read_and_report_via_usb_MT6701_eeprom_values();
    
    if(ENFORCE_MT6701_EEPROM_DEFAULTS){
        bool eeprom_values_updated = false;
        
        if(enforce_abz_mux_by_key(DEFAULT_MT6701_ABZ_MUX_KEY)){
            eeprom_values_updated = true;
        }
        if(enforce_direction_by_key(DEFAULT_MT6701_DIRECTION_KEY)){
            eeprom_values_updated = true;
        }
        if(enforce_out_mode_by_key(DEFAULT_MT6701_OUT_MODE_KEY)){
            eeprom_values_updated = true;
        }
        if(enforce_abz_resolution_by_key(DEFAULT_MT6701_ABZ_RESOLUTION_KEY)){
            eeprom_values_updated = true;
        }

        if(eeprom_values_updated){
            if(PROGRAM_MT6701_EEPROM){
            write_eeprom_programming_sequence();
            printf("EEPROM PROGRAMMING COMPLETE. UPDATED EEPROM VALUES:\n");
            }
            read_and_report_via_usb_MT6701_eeprom_values();
        }
    }

    end_i2c_with_MT6701();

    gpio_set_function(MT6701_CHANNEL_A_PIN, GPIO_FUNC_PIO0);
    gpio_set_function(MT6701_CHANNEL_B_PIN, GPIO_FUNC_PIO0);    

    uint offset = pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, sm, offset, MT6701_CHANNEL_A_PIN, 0);

    gpio_put(MT6701_ENCODER_MODE_PIN,false);

}

void print_current_absolute_position(){
    begin_i2c_with_MT6701();
    int32_t current_position = read_current_absolute_position();
    if(current_position == PICO_ERROR_GENERIC){
        printf("ENCODER POSITION READ ERROR\n");
    }
    else{
    float decoded_absolute_position = decode_absolute_position(current_position);
    printf("CURRENT POSITION: %f\n",decoded_absolute_position);
    }
    end_i2c_with_MT6701();
}

int32_t get_MT6701_quadrature_count(){
    return quadrature_encoder_get_count(pio, sm);
}


