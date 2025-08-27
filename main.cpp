#include <random>
#include  <notcurses/notcurses.h>
#include <thread>
#include <chrono>
#include <cstring>
#include <notcurses/notcurses.h>

#include "robot.h"
#include "Sbus.h"

Robot_Struct robot;


const char* angle_to_arrow(float angle) {
	while (angle < 0) angle += 360;
	while (angle >= 360) angle -= 360;

	if (angle < 22.5 || angle >= 337.5) return "↑";
	if (angle < 67.5) return "↗";
	if (angle < 112.5) return "→";
	if (angle < 157.5) return "↘";
	if (angle < 202.5) return "↓";
	if (angle < 247.5) return "↙";
	if (angle < 292.5) return "←";
	return "↖";
}

void draw_bar(struct ncplane* plane, int y, int x, float value, float max_val, int length) {
	int filled = std::round((value / max_val) * length);
	if (filled > length) filled = length;
	if (filled < 0) filled = 0;
	for (int i = 0; i < length; ++i) {
		if (i < filled ) {
			ncplane_putstr_yx(plane, y, x + i, "█");
		} else {
			ncplane_putchar_yx(plane, y, x + i, ' ');
		}
	}
}


std::random_device rd;
std::mt19937 gen(rd());

int main() {
	struct notcurses_options opts = {};
	struct notcurses* nc = notcurses_init(&opts, nullptr);
	if (!nc) return -1;
	struct ncplane* stdplane = notcurses_stdplane(nc);
	notcurses_cursor_disable(nc);


	robot.is_enabled = 1;

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

	float joystickX = 0.0f; // current value
	float joystickY = 0.0f;

	for ( int frame = 0; frame < 100; ++frame) {

		ncplane_erase(stdplane);

		// --- Telemetry Section ---
		ncplane_printf_yx(stdplane, 1, 1, "Robot Telemetry (Frame %d)", frame);
		ncplane_printf_yx(stdplane, 2, 1, "X Vel: %.2f", robot.xVel);
		draw_bar(stdplane, 2, 20, robot.xVel, MAX_WHEEL_SPEED, 20);

		ncplane_printf_yx(stdplane, 3, 1, "Y Vel: %.2f", robot.yVel);
		draw_bar(stdplane, 3, 20, robot.yVel, MAX_WHEEL_SPEED, 20);

		ncplane_printf_yx(stdplane, 4, 1, "Yaw Vel: %.2f", robot.yawVel);
		draw_bar(stdplane, 4, 20, robot.yawVel, MAX_WHEEL_SPEED, 20);

		ncplane_printf_yx(stdplane, 5, 1, "Enabled: %s", robot.is_enabled ? "YES" : "NO");

		// --- Wheel Directions Section ---
		ncplane_printf_yx(stdplane, 7, 1, "Wheel Directions:");
		ncplane_printf_yx(stdplane, 8, 1, "FL: %s (%.1f°)   FR: %s (%.1f°)",
						  angle_to_arrow(robot.lastFLAngle), robot.lastFLAngle,
						  angle_to_arrow(robot.lastFRAngle), robot.lastFRAngle);
		ncplane_printf_yx(stdplane, 9, 1, "BL: %s (%.1f°)   BR: %s (%.1f°)",
						  angle_to_arrow(robot.lastBLAngle), robot.lastBLAngle,
						  angle_to_arrow(robot.lastBRAngle), robot.lastBRAngle);

		notcurses_render(nc);



		std::uniform_real_distribution<float> change(-50.0f, 50.0f);

		joystickX += change(gen);
		joystickY += change(gen);

		// Clamp values
		if (joystickX > SBUS_MAX) joystickX = SBUS_MAX;
		if (joystickX < SBUS_MIN) joystickX = SBUS_MIN;
		if (joystickY > SBUS_MAX) joystickY = SBUS_MAX;
		if (joystickY < SBUS_MIN) joystickY = SBUS_MIN;

		// Assign to robot
		robot.radio_rx.rx_chan[SBUS_L_STICK_X_CHAN] = (int)joystickX;
		robot.radio_rx.rx_chan[SBUS_L_STICK_Y_CHAN] = (int)joystickY;


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

		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	notcurses_stop(nc);
	return 0;
}