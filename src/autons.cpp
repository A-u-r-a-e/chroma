#include "autons.h"
#include "config.h"

using namespace chromatic;

// note that this reference is only valid for this translation unit/file
MotionController &robot = auton_chassis;

void drive_test() {

    odometry.set_pose(Pose({0,0},0));

    robot.move_by(24, 2000, true, true);
}

void turn_test() {}

void left_side() {}

void right_side() {}

void solo_awp() {}

void skills() {}
