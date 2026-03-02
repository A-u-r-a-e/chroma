#pragma once
#include "api.h"
#include "chromatic.hpp"
#include "config.hpp"
#include "subsystems.hpp"
#include "lidar.hpp"

void knock_in(double speed, chromatic::MotionController &pilot);

void drive_test(chromatic::MotionController &pilot);

void turn_test(chromatic::MotionController &pilot);

void circle_drive(chromatic::MotionController &pilot);

void left_both(chromatic::MotionController &pilot);

void right_both(chromatic::MotionController &pilot);

void right_rush(chromatic::MotionController &pilot);

void solo(chromatic::MotionController &pilot);

void skills(chromatic::MotionController &pilot);
