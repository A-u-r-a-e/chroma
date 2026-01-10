#pragma once
#include "api.h"

using task = pros::Task;
using ms = uint32_t;

inline void delay_for(ms dt) {
    pros::Task::delay(dt);
}
inline void delay_until(ms* ref, ms dt) {
    pros::Task::delay_until(ref, dt);
}
inline ms now() {
    return pros::millis();
}

constexpr auto COAST = pros::E_MOTOR_BRAKE_COAST;
constexpr auto BRAKE = pros::E_MOTOR_BRAKE_BRAKE;
constexpr auto HOLD = pros::E_MOTOR_BRAKE_HOLD;

constexpr auto LX = pros::E_CONTROLLER_ANALOG_LEFT_X;
constexpr auto LY = pros::E_CONTROLLER_ANALOG_LEFT_Y;
constexpr auto RX = pros::E_CONTROLLER_ANALOG_RIGHT_X;
constexpr auto RY = pros::E_CONTROLLER_ANALOG_RIGHT_Y;

constexpr auto L1 = pros::E_CONTROLLER_DIGITAL_L1;
constexpr auto L2 = pros::E_CONTROLLER_DIGITAL_L2;
constexpr auto R1 = pros::E_CONTROLLER_DIGITAL_R1;
constexpr auto R2 = pros::E_CONTROLLER_DIGITAL_R2;

constexpr auto AU = pros::E_CONTROLLER_DIGITAL_UP;
constexpr auto AD = pros::E_CONTROLLER_DIGITAL_DOWN;
constexpr auto AL = pros::E_CONTROLLER_DIGITAL_LEFT;
constexpr auto AR = pros::E_CONTROLLER_DIGITAL_RIGHT;

constexpr auto BA = pros::E_CONTROLLER_DIGITAL_A;
constexpr auto BB = pros::E_CONTROLLER_DIGITAL_B;
constexpr auto BX = pros::E_CONTROLLER_DIGITAL_X;
constexpr auto BY = pros::E_CONTROLLER_DIGITAL_Y;
