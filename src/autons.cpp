#include "autons.h"

using namespace chromatic;

void drive_test() {

    odometry.set_pose(Pose({0,0},0));

    auton_controller.move_by(24, 2000, true, true);
}

void turn_test() {}

void left_side() {}

void right_side() {}

void solo_awp() {}

void skills() {}
