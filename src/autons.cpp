#include "autons.h"
#include "config.h"
#include "subsystems.h"

using namespace chromatic;

// please provide the stuff to use
void drive_test(EncodersIMU &odom, MotionController &pilot) {
    ms prev = now();
    pilot.move_by(24, 2000);
    pros::lcd::print(5, "1: %dms", now() - prev);
    prev = now();
    pilot.move_by(-36, 2000);
    pros::lcd::print(6, "2: %dms", now() - prev);
    prev = now();
    pilot.move_by(12, 2000);
    pros::lcd::print(7, "3: %dms", now() - prev);
}

void turn_test(EncodersIMU &odom, MotionController &pilot) {
    ms prev = now();
    pilot.turn_to(to_rad(180), 2000, false, DIR::CCW);
    pros::lcd::print(1, "1: %dms", now() - prev);
    prev = now();
    pilot.turn_to(to_rad(270), 2000, false, DIR::CCW);
    pros::lcd::print(2, "2: %dms", now() - prev);
    prev = now();
    pilot.turn_to(to_rad(90), 2000, false, DIR::CW);
    pros::lcd::print(3, "3: %dms", now() - prev);
    prev = now();
    pilot.turn_to(0, 2000, false);
    pros::lcd::print(4, "4: %dms", now() - prev);
    prev = now();
}

void move_test(EncodersIMU &odom, MotionController &pilot) {
    ms prev = now();
    pilot.move_to(Pose{{24, -24},  PI / 2}, SIGN::POSITIVE, 4000, false, true);
    pros::lcd::print(1, "M1: %dms", now() - prev);
    prev = now();
    pilot.move_to(Pose{{48, 0}, PI}, SIGN::NEGATIVE, 4000, false, true);
    pros::lcd::print(2, "M2: %dms", now() - prev);
    prev = now();
}

void full_test(EncodersIMU &odom, MotionController &pilot) {
    // move_test(odom, pilot);
    drive_test(odom, pilot);
    turn_test(odom, pilot);
}

void left_side(EncodersIMU &odom, MotionController &pilot) {
    odometry.set_pose(Pose{{0, 0}, to_rad(340)});
    loader.retract();
    body_state = Body::I_STORAGE;
    pilot.move_by(24, 2000, true);
    loader_state = Pneumatic::EXTENDED;
    // pilot.move_by(5, 1000, false);
    delay_for(5000);
}

void right_side(EncodersIMU &odom, MotionController &pilot) {}

void solo_awp(EncodersIMU &odom, MotionController &pilot) {}

void skills(EncodersIMU &odom, MotionController &pilot) {}
