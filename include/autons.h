#pragma once
#include "api.h"
#include "chromatic.hpp"
#include "config.h"
#include "subsystems.h"

void drive_test(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void turn_test(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void circle_drive(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void left_6_1(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void left_7_0(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void right_6_1(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void right_7_0(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void solo_single(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void solo_double(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);
void solo_autism(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);

void skills(chromatic::EncodersIMU &odom, chromatic::MotionController &pilot);
