// robotCodePerf.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <iostream>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "motor_message.h"
#include "math_ops.h"
#include "Sbus.h"

// TODO: Reference additional headers your program requires here.

#define _2pi 6.28318530718f
#define pi 3.14159265359f
#define deg2rad 0.01745329251f


#define MAX_WHEEL_SPEED 200.0f
#define PIVOT_KP 1.0f
#define PIVOT_KD 0.025f
#define WHEEL_KD 0.04f

#define FL_FLIP -1.0f
#define FR_FLIP 1.0f
#define BL_FLIP -1.0f
#define BR_FLIP 1.0f


#define IMU_DT 0.005f
#define enableCutoff 200
#define zeroCutoff 200
#define headlessCutoff 200
#define enableChan 4
#define zeroChan 5
#define headlessChan 6
#define xChan 2
#define yChan 1
#define yawChan 3
#define spdLimChan 7
#define xMin 200
#define xMax 1800
#define yMin 200
#define yMax 1800
#define yawMin 200
#define yawMax 1800
#define spdMin 172
#define spdMax 1800

typedef struct {
	float roll, pitch, yaw;
	float roll_vel, pitch_vel, yaw_vel;
	int imu_update_counter, print_counter;
	Sbus_Struct radio_rx;
	int is_enabled;
	
	joint_state FL_P_STATE,FR_P_STATE,BL_P_STATE,BR_P_STATE,FL_W_STATE,FR_W_STATE,BL_W_STATE,BR_W_STATE;
	joint_control FL_P_CTRL,FR_P_CTRL,BL_P_CTRL,BR_P_CTRL,FL_W_CTRL,FR_W_CTRL,BL_W_CTRL,BR_W_CTRL;
	pivot_cmd FL_P_CMD,FR_P_CMD,BL_P_CMD,BR_P_CMD;
	pwm_cmd FL_W_PWM,FR_W_PWM,BL_W_PWM,BR_W_PWM;
	
	int is_headless;
	float headlesAngle;

	float lastBLAngle, lastBRAngle, lastFLAngle, lastFRAngle;
	
	float cur_max_wheel_speed;
	float xVel, yVel, yawVel;

	float yawGpa, lastYawErr, dYawErr, yawErr, yawCelCL;
	int is_holding;
	int sendCounter;
	int momenteryHigh;

} Robot_Struct;

void control(Robot_Struct *robot);
void debug_print(Robot_Struct *robot);
pivot_cmd process_goal_angle(float cur_angle_abs, float goal);
void swerve(Robot_Struct* robot);
float wrapError(float cur, float goal, float max, float min);
void update_states(Robot_Struct* robot);
