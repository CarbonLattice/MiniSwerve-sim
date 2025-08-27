#include "robot.h"

pivot_cmd process_goal_angle(float cur_angle_abs, float goal) {
	pivot_cmd goal_cmd;
	
	float cur_angle = cur_angle_abs;
	while (cur_angle < -+_2pi) {
		cur_angle+= _2pi;
	}
	while (cur_angle > 0.0f) {
		cur_angle-= _2pi;
	}
	float err1 = goal-cur_angle;
	float err2 = 0;

	if (err1 >= 0) {
		err2 = err1 - _2pi;
	}
	else {
		err2 = err1 + _2pi;
	}
	float errUsed = 0;
	if (fabs(err1) < fabs(err2)) {
		errUsed = err1;
	}
	else {
		errUsed = err2;
	}
	if (errUsed < -pi / 2.0f) {
		errUsed += pi;
		goal_cmd.mult = -1.0f;
	}
	else {
		goal_cmd.mult = 1.0f;
	}
	goal_cmd.errUsed = errUsed;
	goal_cmd.angle = (cur_angle_abs + errUsed);
	return goal_cmd;
}


float wrapError(float cur, float goal, float max, float min) {
	float err1 = goal - cur;
	float err2 = 0.0f;

	if (err1 > 0) {
		err2 = -((cur - min) + (max - goal));
	}
	else {
		err2 = ((cur - max) + (goal - min));
	}
	float errUsed = 0.0f;
	if (fabs(err1) < fabs(err2)) {
		errUsed = err1;
	}
	else {
		errUsed = err2;
	}
	return errUsed;
}

void swerve(Robot_Struct *robot) {
	float x = robot->xVel;
	float y = robot->yVel;
	float yw = robot->yawVel;

	x = x;
	y = y;
	yw = yw;

	float chassisY = 1.0f;
	float chassisX = 1.0f;

	float ax = x + chassisY* -yw;
	float ay = y + chassisX* -yw;

	float backRightSpeedMatrix = sqrt((ax * ax) + (ay * ay));
	float backRightAngleMatrix = -atan2(ay, ax) / pi * 180.0f-90.0f;
	if (backRightAngleMatrix < 0.0f) {
		backRightAngleMatrix+=360.0f;
	}

	float bx = x - chassisY * -yw;
	float by = y + chassisX * -yw;
	float frontRightSpeedMatrix = sqrt((bx * bx) + (by * by));
	float frontRightAngleMatrix = -atan2(by, bx) / pi * 180.0f-90.0f;
	if (frontRightAngleMatrix < 0.0f) {
		frontRightAngleMatrix+=360.0f;
	}



	float cx = x - chassisY * -yw;
	float cy = y + chassisX * -yw;
	float frontLeftSpeedMatrix = sqrt((cx * cx) + (cy * cy));
	float frontLeftAngleMatrix = -atan2(cy, cx) / pi * 180.0f - 90.0f;
	if (frontLeftAngleMatrix < 0.0f) {
		frontLeftAngleMatrix += 360.0f;
	}

	float dx = x - chassisY * -yw;
	float dy = y + chassisX * -yw;
	float backLeftSpeedMatrix = sqrt((dx * dx) + (dy * dy));
	float backLeftAngleMatrix = -atan2(dy, dx) / pi * 180.0f - 90.0f;
	if (backLeftAngleMatrix < 0.0f) {
		backLeftAngleMatrix += 360.0f;
	}

	float backRightSpeed = backRightSpeedMatrix;
	float backLeftSpeed = backLeftSpeedMatrix;
	float frontRightSpeed = frontRightSpeedMatrix;
	float frontLeftSpeed = frontLeftSpeedMatrix;

	float backRightAngle = backRightAngleMatrix;
	float backLeftAngle = backLeftAngleMatrix;
	float frontRightAngle = frontRightAngleMatrix;
	float frontLeftAngle = frontLeftAngleMatrix;

	float max = 0.0f;
	float magBR =  fabsf(backRightSpeed);
	float magBl =  fabsf(backLeftSpeed);
	float magFR =  fabsf(frontRightSpeed);
	float magFL =  fabsf(frontLeftSpeed);

	if(magBR>max)
		max = magBR;
	if(magBl>max)
		max = magBl;
	if(magFR>max)
		max = magFR;
	if(magFL>max)
		max = magFL;
	if (max > 1.0f) {
		backRightSpeed/=max;
		backLeftSpeed/=max;
		frontRightSpeed/=max;
		frontLeftSpeed/=max;
	}
	if (x==0.0f && y==0.0f && fabsf(yw) < 0.01f) {
		backLeftAngle=robot->lastBLAngle;
		backRightAngle=robot->lastBRAngle;
		frontLeftAngle=robot->lastFLAngle;
		frontRightAngle=robot->lastFRAngle;


		backRightSpeed=0.0f;
		backLeftSpeed=0.0f;
		frontRightSpeed=0.0f;
		frontLeftSpeed=0.0f;
	}

	robot->lastBLAngle=backLeftAngle;
	robot->lastBRAngle=backRightAngle;
	robot->lastFRAngle=frontRightAngle;
	robot->lastFLAngle=frontLeftAngle;

	//if (!robot->momenteryHigh || !robot->is_headless) {
		//if(fabsf(robot->))
	//}

	float FL_ANG_CMD = -frontLeftAngle * deg2rad;
	float FR_ANG_CMD = -frontRightAngle * deg2rad;
	float BL_ANG_CMD = -backLeftAngle * deg2rad;
	float BR_ANG_CMD = -backRightAngle * deg2rad;

	robot->FL_P_CMD = process_goal_angle(robot->FL_P_STATE.p, FL_ANG_CMD);
	robot->FR_P_CMD = process_goal_angle(robot->FR_P_STATE.p, FR_ANG_CMD);
	robot->BL_P_CMD = process_goal_angle(robot->BL_P_STATE.p, BL_ANG_CMD);
	robot->BR_P_CMD = process_goal_angle(robot->BR_P_STATE.p, BR_ANG_CMD);

	float FL_W_CMD = frontLeftSpeed * robot->FL_P_CMD.mult * robot->cur_max_wheel_speed * cos(robot->FL_P_CMD.errUsed);
	float FR_W_CMD = frontRightSpeed * robot->FR_P_CMD.mult * robot->cur_max_wheel_speed * cos(robot->FR_P_CMD.errUsed);
	float BL_W_CMD = backLeftSpeed * robot->BL_P_CMD.mult * robot->cur_max_wheel_speed * cos(robot->BL_P_CMD.errUsed);
	float BR_W_CMD = backRightSpeed * robot->BR_P_CMD.mult * robot->cur_max_wheel_speed * cos(robot->BR_P_CMD.errUsed);

	robot->FL_P_CTRL.p_des = robot->FL_P_CMD.angle;
	robot->FR_P_CTRL.p_des = robot->FR_P_CMD.angle;
	robot->BL_P_CTRL.p_des = robot->BL_P_CMD.angle;
	robot->BR_P_CTRL.p_des = robot->BR_P_CMD.angle;

	robot->FL_W_CTRL.v_des = FL_W_CMD * FL_FLIP;
	robot->FR_W_CTRL.v_des = FR_W_CMD * FR_FLIP;
	robot->BL_W_CTRL.v_des = BL_W_CMD * BL_FLIP;
	robot->BR_W_CTRL.v_des = BR_W_CMD * BR_FLIP;

	

}	


void control(Robot_Struct *robot) {
	update_states(robot);
	if (robot->is_enabled) {
		swerve(robot);
	}
	else {
	}
}

void update_states(Robot_Struct* robot) {
	robot->xVel = robot->radio_rx.stick_r_x;
	robot->yVel = robot->radio_rx.stick_r_y;
	robot->yawVel = 0.2f*robot->radio_rx.stick_l_x;
	
	robot->FL_P_CTRL.kp = PIVOT_KP;
	robot->FL_P_CTRL.kd	= PIVOT_KD;
	robot->FR_P_CTRL.kp = PIVOT_KP;
	robot->FR_P_CTRL.kd	= PIVOT_KD;
	robot->BL_P_CTRL.kp = PIVOT_KP;
	robot->BL_P_CTRL.kd	= PIVOT_KD;
	robot->BR_P_CTRL.kp = PIVOT_KP;
	robot->BR_P_CTRL.kd	= PIVOT_KD;

	robot->FL_W_CTRL.kd = WHEEL_KD;
	robot->FR_W_CTRL.kd = WHEEL_KD;
	robot->BL_W_CTRL.kd = WHEEL_KD;
	robot->BR_W_CTRL.kd = WHEEL_KD;

	//if (robot->radio_rx.toggle_right > 0) {
		//if(robot->is_headless==0){
			//robot->is_headless=1;
			//robot->headlesAngle=robot->yaw;
		//}
	//}
	//else {
		robot->is_headless = 0;
	//}
	robot->cur_max_wheel_speed = MAX_WHEEL_SPEED;
	if (robot->is_headless == 1) {
		float curYawIMU = (robot->yaw-robot->headlesAngle)/360.0*2*pi;
		while (curYawIMU < 0.0f) {
			curYawIMU += 2.0f * pi;
			}
		float newYVel = robot->xVel*sin(curYawIMU) + robot->yVel*cos(curYawIMU);
		float newXVel = robot->xVel*cos(curYawIMU) - robot->yVel*sin(curYawIMU);

		robot ->yVel = newYVel;
		robot ->xVel = newXVel;
	}
}
	
void debug_print(Robot_Struct* robot) {
	//printf("Orientation: %f %f %f $f %f %f\r", robot->roll, robot->pitch, robot->yaw, robot->xVel, robot->yVel, robot->yawVel);
	printf("\rFLP:%.3f FLW%.3f FRP:%.3f FRW:%.3f BLP:%.3f BLW:%.3f BRP:%.3f BRW:%.3f	",
	robot->FL_P_CTRL.v_des, robot->FL_W_CTRL.v_des, robot->FR_P_CTRL.v_des, robot->FR_W_CTRL.v_des,
	robot->BL_P_CTRL.v_des, robot->BL_W_CTRL.v_des, robot->BR_P_CTRL.v_des, robot->BR_W_CTRL.v_des);
	fflush(stdout);
	//printf("%d %d %d\r\n", robot->radio_rx.switch_momentary, robot->radio_rx.left_select, robot->radio_rx.right_select);
}