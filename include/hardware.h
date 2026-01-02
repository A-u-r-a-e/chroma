#pragma once
#include "main.h"

extern const ms OP_POLL_RATE;

extern pros::Controller master;
extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;

extern pros::Motor intake;
extern pros::Motor storage;
extern pros::Motor outtake;
extern pros::adi::Pneumatics ear;
extern pros::adi::Pneumatics loader;

extern pros::IMU inertial;
extern pros::Distance lidar;
extern pros::Optical light;
