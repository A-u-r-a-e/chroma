#pragma once
#include "main.h"
#include "chromatic.hpp"
#include "hardware.h"

extern double LIDAR_RANGE;
extern ms auton_pollrate;
extern ms op_pollrate;
extern chromatic::Differential drivetrain;
extern chromatic::EncodersIMU odometry;
extern chromatic::PID fwd_pid;
extern chromatic::PID turn_pid;
extern chromatic::MotionController auton_controller;
