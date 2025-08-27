
#define NOMINMAX
#include <windows.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include "robot.h"
#include "sbus.h"

Robot_Struct robot;


int main() {


	robot.xVel = 0.0;
	robot.yVel = 0.0;
	robot.yawVel = 0.0;
	robot.lastBRAngle = 0.0;
	robot.lastBLAngle = 0.0;
	robot.lastFRAngle = 0.0;
	robot.lastFLAngle = 0.0;
	robot.is_enabled = 1;  // temporarily for testing


	memset(&robot.FL_P_CTRL, 0, sizeof(robot.FL_P_CTRL));
	memset(&robot.FR_P_CTRL, 0, sizeof(robot.FR_P_CTRL));
	memset(&robot.BL_P_CTRL, 0, sizeof(robot.BL_P_CTRL));
	memset(&robot.BR_P_CTRL, 0, sizeof(robot.BR_P_CTRL));
	memset(&robot.FL_W_CTRL, 0, sizeof(robot.FL_W_CTRL));
	memset(&robot.FR_W_CTRL, 0, sizeof(robot.FR_W_CTRL));
	memset(&robot.BL_W_CTRL, 0, sizeof(robot.BL_W_CTRL));
	memset(&robot.BR_W_CTRL, 0, sizeof(robot.BR_W_CTRL));

	bool running = true;


	while (running) {
		for (int i = 0; i < SBUS_NUM_CHAN; i++) {
			robot.radio_rx.rx_chan[i] = SBUS_MIN;
		}


		int rX = SBUS_MID;
		int rY = SBUS_MID;
		int lX = SBUS_MID;
		int lY = SBUS_MID;

		if (GetAsyncKeyState('W') & 0x8000) rY = SBUS_MAX;
		if (GetAsyncKeyState('S') & 0x8000) rY = SBUS_MIN;
		if (GetAsyncKeyState('A') & 0x8000) rX = SBUS_MIN;
		if (GetAsyncKeyState('D') & 0x8000) rX = SBUS_MAX;

		if (GetAsyncKeyState('I') & 0x8000) lY = SBUS_MAX;
		if (GetAsyncKeyState('K') & 0x8000) lY = SBUS_MIN;
		if (GetAsyncKeyState('J') & 0x8000) lX = SBUS_MIN;
		if (GetAsyncKeyState('L') & 0x8000) lX = SBUS_MAX;

		robot.radio_rx.rx_chan[SBUS_L_STICK_X_CHAN] = rX;
		robot.radio_rx.rx_chan[SBUS_L_STICK_Y_CHAN] = rY;

		robot.radio_rx.rx_chan[SBUS_R_STICK_X_CHAN] = lX;
		robot.radio_rx.rx_chan[SBUS_R_STICK_Y_CHAN] = lY;

		
		sbus_update(&robot.radio_rx);

		//robot.xVel = rX;
		//robot.yVel = rY;
		//robot.yawVel = lX;



		//std::cout << "\rInputs: xVel=" << robot.xVel << " yVel=" << robot.yVel
			//<< " yawVel=" << robot.yawVel << "      ";
		//std::cout.flush();


		//robot.xVel = sbus.stick_r_x;
		//robot.yVel = sbus.stick_r_y;
		//robot.yawVel = sbus.stick_l_x;




		control(&robot);

		//std::cout << "\rR Stick: (" << robot.radio_rx.stick_r_x << ", " << robot.radio_rx.stick_r_y
		//	<< ")  L Stick: (" << robot.radio_rx.stick_l_x << ", " << robot.radio_rx.stick_l_y << ")   ";
		//std::cout.flush();

		//debug_print(&robot);

		//std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
	std::cout << "Exiting program." << std::endl;
	return 0;
}