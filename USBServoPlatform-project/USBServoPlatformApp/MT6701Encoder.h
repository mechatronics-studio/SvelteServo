#ifndef MT6701_ENCODER
#define MT6701_ENCODER

void report_encoder_eeprom_values();

void print_current_absolute_position();

void initialize_MT6701();

int32_t get_MT6701_quadrature_count();

#endif