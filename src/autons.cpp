#include "autons.h"
#include "config.h"

using namespace chromatic;

// please provide the stuff to use
void drive_test(EncodersIMU &odom, MotionController &pilot) {

    odom.set_pose(Pose({0,0},0));

    pilot.move_by(24, 2000, true, true);
}

void turn_test(EncodersIMU &odom, MotionController &pilot) {}

void left_side(EncodersIMU &odom, MotionController &pilot) {}

void right_side(EncodersIMU &odom, MotionController &pilot) {}

void solo_awp(EncodersIMU &odom, MotionController &pilot) {}

void skills(EncodersIMU &odom, MotionController &pilot) {}
