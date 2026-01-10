#include "autons.h"
#include "config.h"
#include "subsystems.h"

using namespace chromatic;

// please provide the stuff to use
void drive_test(EncodersIMU &odom, MotionController &pilot) {
    ms prev = now();
    double one = pilot.move_by(48, 2000, false);
    pros::lcd::print(5, "1: %dms, %finch", now() - prev, one);
    prev = now();
    double two = pilot.move_by(-36, 2000, false);
    pros::lcd::print(6, "2: %dms, %finch", now() - prev, two);
    prev = now();
    double three = pilot.move_by(-12, 2000, false);
    pros::lcd::print(7, "3: %dms, %finch", now() - prev, three);
}

void turn_test(EncodersIMU &odom, MotionController &pilot) {
    ms prev = now();
    double one = pilot.turn_to(to_rad(180), 2000, false, DIR::CCW);
    pros::lcd::print(1, "1: %dms, %fdeg", now() - prev, to_deg(one));
    prev = now();
    double two = pilot.turn_to(to_rad(270), 2000, false, DIR::CCW);
    pros::lcd::print(2, "2: %dms, %fdeg", now() - prev, to_deg(two));
    prev = now();
    double three = pilot.turn_to(to_rad(90), 2000, false, DIR::CW);
    pros::lcd::print(3, "3: %dms, %fdeg", now() - prev, to_deg(three));
    prev = now();
    double four = pilot.turn_to(0, 2000, false);
    pros::lcd::print(4, "4: %dms, %fdeg", now() - prev, to_deg(four));
    prev = now();
}

void move_test(EncodersIMU &odom, MotionController &pilot) {
    ms prev = now();
    double one = pilot.move_to(Pose{{24, -24},  PI / 2}, SIGN::POSITIVE, 4000, false, true);
    pros::lcd::print(1, "M1: %dms, %finch", now() - prev, one);
    prev = now();
    double two = pilot.move_to(Pose{{48, 0}, PI}, SIGN::NEGATIVE, 4000, false, true);
    pros::lcd::print(2, "M2: %dms, %finch", now() - prev, two);
    prev = now();
}

void full_test(EncodersIMU &odom, MotionController &pilot) {
    // move_test(odom, pilot);
    drive_test(odom, pilot);
    turn_test(odom, pilot);
}

void left_side(EncodersIMU &odom, MotionController &pilot) {
    // odometry.set_pose(Pose{{0, 0}, to_rad(340)});
    // 38, 27.3, 252º, -15.7, 36.7, +40, 9, -29
    loader.retract();
    body_state = Body::I_STORAGE;
    pilot.move_by(24,  4000, 30, false);
    loader_state = Pneumatic::EXTENDED;
    pilot.move_by(14, 4000, -1, false);
    pilot.move_by(-11.7, 4000, 40, false);
    pilot.turn_to(to_rad(112), 4000, false);
    pilot.move_by(-15.7, 4000, 20, false);
    pilot.move_by(54, 4000, -1, true);
    pilot.turn_to(to_rad(156), 4000, true);
    pilot.move_by(9, 4000, 40, true);
    pilot.move_by(-29, 4000, -1, true);
}

void right_side(EncodersIMU &odom, MotionController &pilot) {}

void solo_awp(EncodersIMU &odom, MotionController &pilot) {}

void skills(EncodersIMU &odom, MotionController &pilot) {}
