#ifndef BEZIER_CURVE
#define BEZIER_CURVE
#include "pico/stdlib.h"

typedef struct{
    int32_t P0x;
    int32_t P0y;
    int32_t P1x;
    int32_t P1y;
    int32_t P2x;
    int32_t P2y;
    int32_t P3x;
    int32_t P3y;
} bezier_curve;

int32_t bezier_curve_calculate_y_from_x(bezier_curve input_curve, int32_t x, int32_t x_error_stopping_criteria);


#endif