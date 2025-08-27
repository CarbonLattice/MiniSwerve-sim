#include <notcurses/notcurses.h>
#include <thread>
#include <chrono>
#include <random>
#include <cmath>
#include <iomanip>
#include <sstream>
#include "robot.h"
#include "Sbus.h"

// --- Robot struct ---
Robot_Struct robot;

// --- Constants ---
constexpr int MAX_VEL = 100;
constexpr int BAR_WIDTH = 20;
constexpr int ROBOT_HEIGHT = 5;
constexpr int ROBOT_WIDTH = 9;

// --- Random generator ---
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<> dist(SBUS_MIN, SBUS_MAX);

// --- 32-direction wheel characters ---
const char* wheel_chars[32] = {
    "│","╱","╱","╱","─","╲","╲","╲","│","╱","╱","╱","─","╲","╲","╲",
    "│","╱","╱","╱","─","╲","╲","╲","│","╱","╱","╱","─","╲","╲","╲"
};

// --- Map angle to wheel character ---
const char* angle_to_wheel(float angle){
    while(angle < 0) angle += 360;
    while(angle >= 360) angle -= 360;
    int index = static_cast<int>((angle / 360.0f) * 32.0f);
    return wheel_chars[index % 32];
}

// --- Draw colored bar ---
void draw_bar(struct ncplane* plane, int y, int x, float value, float max_val, int length){
    if(value < 0) value = 0;
    if(value > max_val) value = max_val;

    int filled = static_cast<int>((value / max_val) * length);
    int r = static_cast<int>(255.0f * (value / max_val));
    int g = static_cast<int>(255.0f * (1.0f - value / max_val));
    ncplane_set_bg_rgb(plane, 0x000000);

    for(int i=0;i<length;i++){
        if(i < filled){
            ncplane_set_fg_rgb(plane, (r << 16) | (g << 8));
            ncplane_putstr_yx(plane, y, x + i, "█");
        } else {
            ncplane_set_fg_rgb(plane, 0x555555);
            ncplane_putstr_yx(plane, y, x + i, "░");
        }
    }
}

// --- Draw high-res robot with wheel colors based on speed ---
void draw_robot(struct ncplane* plane, int y, int x){
    // Robot body
    ncplane_putstr_yx(plane, y, x + 2, "┌─────┐");
    ncplane_putstr_yx(plane, y + 1, x + 2, "│     │");
    ncplane_putstr_yx(plane, y + 2, x + 2, "│     │");
    ncplane_putstr_yx(plane, y + 3, x + 2, "└─────┘");

    // Compute wheel colors based on speed (normalized 0–255)
    auto color_from_speed = [](float speed){
        float magnitude = std::fabs(speed)/MAX_VEL;
        int r = static_cast<int>(255 * magnitude);
        int g = static_cast<int>(255 * (1.0f - magnitude));
        return (r << 16) | (g << 8);
    };

    // Front wheels
    ncplane_set_fg_rgb(plane, color_from_speed(robot.xVel));
    ncplane_putstr_yx(plane, y, x, angle_to_wheel(robot.lastFLAngle));       // FL
    ncplane_putstr_yx(plane, y, x + 8, angle_to_wheel(robot.lastFRAngle));   // FR

    // Rear wheels
    ncplane_set_fg_rgb(plane, color_from_speed(robot.yVel));
    ncplane_putstr_yx(plane, y + 3, x, angle_to_wheel(robot.lastBLAngle));   // BL
    ncplane_putstr_yx(plane, y + 3, x + 8, angle_to_wheel(robot.lastBRAngle)); // BR

    // Numerical angles
    ncplane_set_fg_rgb(plane, 0xFFFFFF);
    ncplane_printf_yx(plane, y - 1, x, "%.0f", robot.lastFLAngle);
    ncplane_printf_yx(plane, y - 1, x + 8, "%.0f", robot.lastFRAngle);
    ncplane_printf_yx(plane, y + 4, x, "%.0f", robot.lastBLAngle);
    ncplane_printf_yx(plane, y + 4, x + 8, "%.0f", robot.lastBRAngle);
}

// --- Interpolate angles smoothly ---
float interpolate_angle(float current, float target, float factor){
    float diff = target - current;
    if(diff > 180) diff -= 360;
    if(diff < -180) diff += 360;
    return current + diff * factor;
}

int main(){
    struct notcurses_options opts{};
    struct notcurses* nc = notcurses_init(&opts, nullptr);
    if(!nc) return -1;
    struct ncplane* stdplane = notcurses_stdplane(nc);
    notcurses_cursor_disable(nc);

    robot.is_enabled = 1;
    robot.xVel = robot.yVel = robot.yawVel = 0.0f;
    robot.lastFLAngle = robot.lastFRAngle = robot.lastBLAngle = robot.lastBRAngle = 0.0f;

    bool running = true;
    int frame = 0;

    while(running){
        ncplane_erase(stdplane);

        // --- Telemetry ---
        ncplane_printf_yx(stdplane, 1, 2, "X Vel:");
        draw_bar(stdplane, 1, 10, robot.xVel, MAX_VEL, BAR_WIDTH);
        ncplane_printf_yx(stdplane, 2, 2, "Y Vel:");
        draw_bar(stdplane, 2, 10, robot.yVel, MAX_VEL, BAR_WIDTH);
        ncplane_printf_yx(stdplane, 3, 2, "Yaw Vel:");
        draw_bar(stdplane, 3, 10, robot.yawVel, MAX_VEL, BAR_WIDTH);

        // --- Simulate controller input ---
        robot.radio_rx.rx_chan[SBUS_L_STICK_X_CHAN] = dist(gen);
        robot.radio_rx.rx_chan[SBUS_L_STICK_Y_CHAN] = dist(gen);
        robot.radio_rx.rx_chan[SBUS_R_STICK_X_CHAN] = dist(gen);
        robot.radio_rx.rx_chan[SBUS_R_STICK_Y_CHAN] = dist(gen);
        sbus_update(&robot.radio_rx);

        // --- Update robot logic ---
        control(&robot);

        // --- Smoothly interpolate wheel angles for animation ---
        float factor = 0.2f; // interpolation factor
        robot.lastFLAngle = interpolate_angle(robot.lastFLAngle, robot.FL_P_CMD.angle, factor);
        robot.lastFRAngle = interpolate_angle(robot.lastFRAngle, robot.FR_P_CMD.angle, factor);
        robot.lastBLAngle = interpolate_angle(robot.lastBLAngle, robot.BL_P_CMD.angle, factor);
        robot.lastBRAngle = interpolate_angle(robot.lastBRAngle, robot.BR_P_CMD.angle, factor);

        // --- Draw robot view ---
        draw_robot(stdplane, 6, 5);

        // Render frame
        notcurses_render(nc);

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 20 FPS
        frame++;
        if(frame > 400) running = false;
    }

    notcurses_stop(nc);
    return 0;
}
