cmake_minimum_required(VERSION 4.0)
project(MiniSwerve_sim_linux)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(MiniSwerve_sim_linux
        constants.h
        Header.h
        main.cpp
        math_ops.cpp
        math_ops.h
        motor_message.h
        robot.cpp
        robot.h
        Sbus.cpp
        Sbus.h)
