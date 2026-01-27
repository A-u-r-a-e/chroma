#include "autons.h"
#include "chromatic/shorthands.hpp"
#include "config.h"
#include "subsystems.h"

using namespace chromatic;

// please provide the stuff to use
void drive_test(EncodersIMU &odom, MotionController &pilot) {
    ms prev = now();
    double one = pilot.move_by(48, 3000);
    pros::lcd::print(1, "1: %dms, %f inch", now() - prev, one);
    // delay_for(1000);
    prev = now();
    double two = pilot.move_by(-48, 3000);
    pros::lcd::print(2, "2: %dms, %f inch", now() - prev, two);
    // delay_for(1000);
    prev = now();
    double three = pilot.move_by(24, 3000);
    pros::lcd::print(3, "3: %dms, %f inch", now() - prev, three);
    // delay_for(1000);
}

void turn_test(EncodersIMU &odom, MotionController &pilot) {
    ms prev = now();
    double one = pilot.turn_to(135, 4000);
    pros::lcd::print(4, "1: %dms, %fdeg", now() - prev, to_deg(one));
    // delay_for(1000);
    prev = now();
    double two = pilot.turn_to(315, 4000);
    pros::lcd::print(5, "2: %dms, %fdeg", now() - prev, to_deg(two));
    // delay_for(1000);
    prev = now();
    double three = pilot.turn_to(90, 4000);
    pros::lcd::print(6, "3: %dms, %fdeg", now() - prev, to_deg(three));
    // delay_for(1000);
    prev = now();
    double four = pilot.turn_to(0, 4000);
    pros::lcd::print(7, "4: %dms, %fdeg", now() - prev, to_deg(four));
    prev = now();
}

void circle_drive(EncodersIMU &odom, MotionController &pilot) {
    hook_state = Pneumatic::RETRACTED;
    pilot.move_by(60, 3000, -1, true, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(270, 1000, true, DIR::EITHER, {to_rad(30), to_rad(90), 40, 0});
    pilot.move_by(60, 3000, -1, true, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(180, 1000, true, DIR::EITHER, {to_rad(30), to_rad(90), 40, 0});
    pilot.move_by(60, 3000, -1, true, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(90, 1000, true, DIR::EITHER, {to_rad(30), to_rad(90), 40, 0});
    pilot.move_by(60, 3000, -1, true, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(0, 1000, true, DIR::EITHER);
}

void left_both(EncodersIMU &odom, MotionController &pilot) {
}

void right_both(EncodersIMU &odom, MotionController &pilot) {
}

void solo_single(EncodersIMU &odom, MotionController &pilot) {
}

void solo_double(EncodersIMU &odom, MotionController &pilot) {
}

void skills(EncodersIMU &odom, MotionController &pilot) {
}
