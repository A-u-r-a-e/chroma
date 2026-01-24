#pragma once
#include "api.h"
#include "chromatic.hpp"
#include "config.h"
#include "subsystems.h"

void drive_test(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void turn_test(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void circle_drive(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void left_both(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void right_both(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void solo_single(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void solo_double(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void skills(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);
