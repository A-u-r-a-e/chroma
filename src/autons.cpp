#include "autons.h"
#include "chromatic/shorthands.hpp"
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
    pilot.move_by(20,  2000, 30, true);
    // loader_state = Pneumatic::EXTENDED;
    pilot.move_by(7.3,  1000, 30, false);
    // loader_state = Pneumatic::RETRACTED;
    // delay_for(200);
    pilot.turn_to(to_rad(112), 1000, false);
    pilot.move_by(-14.7, 2000, 20, true);
    body_state = Body::S_MIDDLE;
    delay_for(500);
    body_state = Body::I_STORAGE;
    pilot.move_by(54, 2000, -1, true);
    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(to_rad(156), 750, true);
    pilot.move_by(10, 750, -1, true);
    delay_for(1000);
    pilot.move_by(-30, 2000, -1, true);
    body_state = Body::S_FULL;
    delay_for(1000);
    pilot.move_by(12, 2000, -1, true);
    pilot.turn_to(to_rad(111), 1000, true);
    pilot.move_by(-15, 2000, -1, true);
    pilot.turn_to(to_rad(156), 1000, true);
    hook_state = Pneumatic::RETRACTED;
    pilot.move_by(-20, 2000, -1, true);
}

void right_side(EncodersIMU &odom, MotionController &pilot) {}

void solo_awp(EncodersIMU &odom, MotionController &pilot) {
    // 48, 270º, 14, -31.7
    // 6.5, 155º, 28, 180º
    // 46, 220, -17
    // 52.5, 270º, 10, -31.7
    pilot.move_by(48, 2000, -1, true);
    pilot.turn_to(to_rad(270), 1000, true);
    pilot.move_by(14, 2000, -1, true);
    pilot.move_by(-31.7, 2000, -1, true);
    pilot.move_by(6.5, 2000, -1, true);
    pilot.turn_to(to_rad(155), 1000, true);
    pilot.move_by(28, 2000, -1, true);
    pilot.turn_to(to_rad(180), 1000, true);
    pilot.move_by(46, 2000, -1, true);
    pilot.turn_to(to_rad(220), 1000, true);
    pilot.move_by(-17, 2000, -1, true);
    pilot.move_by(52.5, 2000, -1, true);
    pilot.turn_to(to_rad(270), 1000, true);
    pilot.move_by(10, 2000, -1, true);
    pilot.move_by(-31.7, 2000, -1, true);

}

void skills(EncodersIMU &odom, MotionController &pilot) {}
