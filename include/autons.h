#pragma once
#include "api.h"
#include "chromatic.hpp"
#include "config.h"
#include "subsystems.h"

void drive_test(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void turn_test(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void full_test(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void left_side(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void right_side(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void skills(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void solo_awp(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);
