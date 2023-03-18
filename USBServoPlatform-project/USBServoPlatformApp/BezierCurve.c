#include "BezierCurve.h"

int64_t int32_max_value = 2147483647;

bezier_curve bezier_curve_circular_buffer[10];
uint32_t buffer_start_index = 0;
uint32_t buffer_end_index = 0;



int32_t lerp(int32_t a, int32_t b, uint32_t u){
    int64_t a64 = a;
    int64_t b64 = b;
    int64_t u64 = u;
    int32_t result = ((int32_max_value - u64)*a64 + u64*b64)/int32_max_value;
    return result;
}

int32_t bezier_curve_calculate_x_from_u(bezier_curve input_curve, int32_t u){
    int32_t interp1 = lerp(input_curve.P0x,input_curve.P1x,u);
    int32_t interp2 = lerp(input_curve.P1x,input_curve.P2x,u);
    int32_t interp3 = lerp(input_curve.P2x,input_curve.P3x,u);

    int32_t interp4 = lerp(interp1,interp2,u);
    int32_t interp5 = lerp(interp2,interp3,u);

    int32_t result = lerp(interp4,interp5,u);
    return result;
}

int32_t bezier_curve_calculate_y_from_u(bezier_curve input_curve, int32_t u){
    int32_t interp1 = lerp(input_curve.P0y,input_curve.P1y,u);
    int32_t interp2 = lerp(input_curve.P1y,input_curve.P2y,u);
    int32_t interp3 = lerp(input_curve.P2y,input_curve.P3y,u);

    int32_t interp4 = lerp(interp1,interp2,u);
    int32_t interp5 = lerp(interp2,interp3,u);

    int32_t result = lerp(interp4,interp5,u);
    return result;
}

int32_t bezier_curve_calculate_y_from_x(bezier_curve input_curve, int32_t x, int32_t x_error_stopping_criteria){
    int32_t u_upper = int32_max_value;
    int32_t u_lower = 0;
    int32_t u_guess = (u_upper + u_lower)/2;
    int32_t x_from_u_guess = bezier_curve_calculate_x_from_u(input_curve, u_guess);
    int32_t x_error = x_from_u_guess - x;
    while(x_error > x_error_stopping_criteria){
        if(x_error > 0){
            u_upper = u_guess;
        }
        else{
            u_lower = u_guess;
        }
        u_guess = (u_upper-u_lower)/2 + u_lower;
        x_from_u_guess = bezier_curve_calculate_x_from_u(input_curve, u_guess);
        x_error = x_from_u_guess - x;
    }

    int32_t y = bezier_curve_calculate_y_from_u(input_curve, u_guess);
}
