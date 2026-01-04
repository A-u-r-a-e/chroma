#pragma once
#include "main.h"
#include "hardware.h"

extern chromatic::Differential drivetrain;
extern chromatic::EncodersIMU odometry;
extern chromatic::PID fwd_pid;
extern chromatic::PID turn_pid;
extern chromatic::MotionController auton_controller;
