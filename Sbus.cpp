#include "Sbus.h"

float normalize_chan(int cur, int min, int max) {
	return (float)(cur - min) / (float)(max - min);
}

void sbus_update(Sbus_Struct* sbus) {
	if (sbus->rx_chan[SBUS_L_TOG_CHAN] > SBUS_MIN) {
		sbus->is_enabled = 1;
	}
	else {
		sbus->is_enabled = 0;
	}
	if (sbus->rx_chan[SBUS_R_TOG_CHAN] > SBUS_MID) {
		sbus->toggle_right = 1;
	}
	else {
		sbus->toggle_right = 0;
	}
	if (sbus->rx_chan[SBUS_SWITCH_MOM_CHAN] > SBUS_MIN) {
		sbus->switch_momentary = 1;
	}
	else {
		sbus->switch_momentary = 0;
	}
	if (sbus->rx_chan[SBUS_L_SELECT_CHAN] > SBUS_MID) {
		sbus->left_select = 2;
	}
	else if (sbus->rx_chan[SBUS_L_SELECT_CHAN] > SBUS_MIN) {
		sbus->left_select = 1;
	}
	else {
		sbus->left_select = 0;
	}
	if (sbus->rx_chan[SBUS_R_SELECT_CHAN] > SBUS_MID) {
		sbus->right_select = 2;
	}
	else if (sbus->rx_chan[SBUS_R_SELECT_CHAN] > SBUS_MIN) {
		sbus->right_select = 1;
	}
	else {
		sbus->right_select = 0;
	}

	sbus->knob_r = normalize_chan(sbus->rx_chan[SBUS_KNOB_CHAN], SBUS_MIN, SBUS_MAX);
	if (sbus->left_select > 1) {
		sbus->stick_l_x = (normalize_chan(sbus->rx_chan[SBUS_L_STICK_X_CHAN], SBUS_MIN, SBUS_MAX) - 0.5f) * 2.0f;
		sbus->stick_l_y = (normalize_chan(sbus->rx_chan[SBUS_L_STICK_Y_CHAN], SBUS_MIN, SBUS_MAX) - 0.5f) * 2.0f;
		sbus->stick_r_x = (normalize_chan(sbus->rx_chan[SBUS_R_STICK_X_CHAN], SBUS_MIN, SBUS_MAX) - 0.5f) * 2.0f;
		sbus->stick_r_y = (normalize_chan(sbus->rx_chan[SBUS_R_STICK_Y_CHAN], SBUS_MIN, SBUS_MAX) - 0.5f) * 2.0f;
	}
	else {
		sbus->stick_r_x = (normalize_chan(sbus->rx_chan[SBUS_L_STICK_X_CHAN], SBUS_MIN, SBUS_MAX) - 0.5f) * 2.0f;
		sbus->stick_r_y = (normalize_chan(sbus->rx_chan[SBUS_L_STICK_Y_CHAN], SBUS_MIN, SBUS_MAX) - 0.5f) * 2.0f;
		sbus->stick_l_x = (normalize_chan(sbus->rx_chan[SBUS_R_STICK_X_CHAN], SBUS_MIN, SBUS_MAX) - 0.5f) * 2.0f;
		sbus->stick_l_y = (normalize_chan(sbus->rx_chan[SBUS_R_STICK_Y_CHAN], SBUS_MIN, SBUS_MAX) - 0.5f) * 2.0f;
	}
	if (fabsf(sbus->stick_l_x) < STICK_DEADBAND) {
		sbus->stick_l_x=0.0f;
	}
	if (fabsf(sbus->stick_r_x) < STICK_DEADBAND) {
		sbus->stick_r_x = 0.0f;
	}
	if (fabsf(sbus->stick_l_y) < STICK_DEADBAND) {
		sbus->stick_l_y = 0.0f;
	}
	if (fabsf(sbus->stick_r_y) < STICK_DEADBAND) {
		sbus->stick_r_y = 0.0f;
	}
}