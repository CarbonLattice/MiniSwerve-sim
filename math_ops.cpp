#include "math_ops.h"

int float_to_uint(float x, float x_min, float x_max, int bits) {
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}


float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

float sign(float num) {

    //Returns sign of float

    if (num > 0) return 1.0f;

    else return -1.0f;

}

int float_to_rcpwm(float val, float val_min, float val_max, int pwm_min, int pwm_max) {
    // Converts floats to rc pwm integers
    if (val < val_min) val = val_min;
    if (val > val_max) val = val_max;

    float scale = (pwm_max - pwm_min) / (val_max - val_min);

    int pwm_value = pwm_min + (int)((val - val_min) * scale);

    return pwm_value;
}
