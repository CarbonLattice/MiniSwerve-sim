#pragma once
#include "constants.h"
#include <math.h>

typedef struct {
    int rx_chan[SBUS_NUM_CHAN];
    float stick_l_x, stick_l_y, stick_r_x, stick_r_y;  //{-1.0, 1.0}
    float knob_r;                              //{0.0, 1.0}
    int is_enabled; //Left Toggle Switch                //0 or 1
    int toggle_right; //Right Toggle Switch //0 or 1
    int switch_momentary; //Momentary Switch //0 or 1
    int left_select, right_select; //Left and right selection switches 0,1,2

} Sbus_Struct;

void sbus_update(Sbus_Struct* sbus);

#define STICK_DEADBAND 0.1f

#define SBUS_MIN 172
#define SBUS_MAX 1811
#define SBUS_MID 992
#define SBUS_STICK_L_Y_MIN 172
#define SBUS_STICK_L_Y_MAX 1911
#define SBUS_STICK_L_X_MIN 172
#define SBUS_STICK_L_X_MAX 1911
#define SBUS_STICK_R_Y_MIN 172
#define SBUS_STICK_R_Y_MAX 1911
#define SBUS_STICK_R_X_MIN 172
#define SBUS_STICK_R_X_MAX 1911
#define SBUS_L_U_MIN 172
#define SBUS_L_U_MAX 1811
#define SBUS_R_L_R_MIN 172
#define SBUS_R_L_R_MID 992
#define SBUS_R_L_R_MAX 1811
#define SBUS_R_L_L_MIN 172
#define SBUS_R_L_L_MID 992
#define SBUS_R_L_L_MAX 1811
#define SBUS_L_KNOB_MIN 172
#define SBUS_L_KNOB_MAX 1811
#define SBUS_R_KNOB_MIN 172
#define SBUS_R_KNOB_MAX 1811

#define SBUS_L_STICK_Y_CHAN 0
#define SBUS_L_STICK_X_CHAN 3
#define SBUS_R_STICK_Y_CHAN 2
#define SBUS_R_STICK_X_CHAN 1
#define SBUS_KNOB_CHAN 10
#define SBUS_SWITCH_MOM_CHAN 8
#define SBUS_L_TOG_CHAN 6
#define SBUS_R_TOG_CHAN 7

#define SBUS_L_SELECT_CHAN 4
#define SBUS_R_SELECT_CHAN 5