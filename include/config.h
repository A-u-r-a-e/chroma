#pragma once
#include "api.h"
#include "chromatic.hpp"

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

extern double LIDAR_RANGE;
extern ms auton_pollrate;
extern ms op_pollrate;
extern chromatic::Differential drivebase;
extern chromatic::EncodersIMU odometry;
extern chromatic::PID fwd_pid;
extern chromatic::PID turn_pid;
extern chromatic::MotionController auton_controller;
