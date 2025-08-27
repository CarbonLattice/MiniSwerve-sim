#pragma once

#include "math.h"

int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
float sign(float num);
int float_to_rcpwm(float val, float val_min, float val_max, int pwm_min, int pwm_max);