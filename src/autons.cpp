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
    double one = pilot.turn_to(30, 3000);
    pros::lcd::print(4, "1: %dms, %fdeg", now() - prev, one);
    // delay_for(1000);
    prev = now();
    double two = pilot.turn_to(90, 3000);
    pros::lcd::print(5, "2: %dms, %fdeg", now() - prev, two);
    // delay_for(1000);
    prev = now();
    double three = pilot.turn_to(180, 3000);
    pros::lcd::print(6, "3: %dms, %fdeg", now() - prev, three);
    // delay_for(1000);
    prev = now();
    double four = pilot.turn_to(0, 3000);
    pros::lcd::print(7, "4: %dms, %fdeg", now() - prev, four);
}

void circle_drive(EncodersIMU &odom, MotionController &pilot) {
    hook_state = Pneumatic::RETRACTED;
    pilot.move_by(60, 3000, true, -1, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(270, 1000, true, false, {to_rad(30), to_rad(90), 40, 0});
    pilot.move_by(60, 3000, true, -1, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(180, 1000, true, false, {to_rad(30), to_rad(90), 40, 0});
    pilot.move_by(60, 3000, true, -1, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(90, 1000, true, false, {to_rad(30), to_rad(90), 40, 0});
    pilot.move_by(60, 3000, true, -1, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(0, 1000, true);
}

void left_both(EncodersIMU &odom, MotionController &pilot) {
}

void right_both(EncodersIMU &odom, MotionController &pilot) {
}

void solo_single(EncodersIMU &odom, MotionController &pilot) {
}

void solo_double(EncodersIMU &odom, MotionController &pilot) {
    pilot.move_by(31, 1000, true, -1, {5, 10, 0, to_rad(-30)});
    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(270, 1000, true, false, {to_rad(15), to_rad(50), 40, 0});
    body_state = Body::PREP_SCORE;
    pilot.move_by(10.6, 800, true);
    // delay_for(300);
    pilot.move_by(-33.1, 1500, true);
    body_state = Body::S_FULL;
    delay_for(1200);
    loader_state = Pneumatic::RETRACTED;
    body_state = Body::PREP_SCORE;
    pilot.turn_to(170, 1000, true, true);
    pilot.move_by(9.5, 1000, true);
    pilot.turn_to(180, 1000, true);
    pilot.move_by(48+6, 1800, true);
    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(225, 600, true);
    pilot.move_by(-17, 800, true);
    body_state = Body::S_MIDDLE;
    delay_for(600);
    body_state = Body::I_STORAGE;
    pilot.move_by(54.3, 1500, true);
    pilot.turn_to(270, 600, true);
    pilot.move_by(-22.5, 1200, true);
    body_state = Body::S_LOW;
    delay_for(300);
    body_state = Body::S_FULL;
    delay_for(1500);
}

void skills(EncodersIMU &odom, MotionController &pilot) {

    const double EXTRA_SPACE = 1;

    auto store_state = [&] {body_state = Body::PREP_SCORE;};
    auto hold_state = [&] {body_state = Body::NOTHING;};

    auto score_then_store = [&](ms dur = 3500) {
        loader_state = Pneumatic::EXTENDED;
        body_state = Body::M_REFRESH;
        delay_for(200);
        body_state = Body::S_FULL;
        delay_for(dur);
        loader_state = Pneumatic::RETRACTED;
        body_state = Body::PREP_SCORE;
    };

    auto knock_in = [&](double speed) {
        pilot.move_by(10, 500, true, speed);
        pilot.move_by(-10, 500, true, speed);
    };

    const MotionController::Chain TURN_TO_FWD = {to_rad(15), to_rad(50), 40, 0};
    const MotionController::Chain TURN_TO_BACK = {to_rad(15), to_rad(50), -40, 0};
    const MotionController::Chain DRIVE_TO_CW =  {5, 10, 0, to_rad(-30)};
    const MotionController::Chain DRIVE_TO_CCW =  {5, 10, 0, to_rad(30)};

    pilot.move_by(31, 1000, true, -1, DRIVE_TO_CW);
    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(270, 1000, true, false, TURN_TO_FWD);
    store_state();
    pilot.move_by(10.7, 800, true);
    delay_for(1500);


    pilot.move_by(-9.7, 2000, true);
    loader_state = Pneumatic::RETRACTED;
    pilot.turn_to(45, 2000, true);
    pilot.move_by((12*sqrt(2)+EXTRA_SPACE*sqrt(2)), 2000, false);
    pilot.turn_to(90, 2000, true);
    pilot.move_by(24*3-2*EXTRA_SPACE, 2000, true);
    pilot.turn_to(135, 2000, true);
    pilot.move_by((12*sqrt(2)+EXTRA_SPACE*sqrt(2)), 2000, false);
    pilot.turn_to(90, 2000, true);
    loader_state = Pneumatic::EXTENDED;


    pilot.move_by(-20.2, 2000, true);
    score_then_store(2500);
    score_then_store(500);
    loader_state = Pneumatic::EXTENDED;
    pilot.move_by(33.7, 2000, true);
    delay_for(1500);
    pilot.move_by(-33.7, 2000, true);
    score_then_store(2500);
    score_then_store(500);
    knock_in(30);
    pilot.move_by(16, 2000, true, -1, DRIVE_TO_CCW);


    pilot.turn_to(180, 1000, true, false);
    pilot.move_by(24*4+1.5, 4000, true, -1, DRIVE_TO_CW);


    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(90, 1000, true, false, TURN_TO_FWD);
    store_state();
    pilot.move_by(18.2, 2000, true);
    delay_for(1500);

    pilot.move_by(-10.2, 2000, true);
    loader_state = Pneumatic::RETRACTED;
    pilot.turn_to(225, 2000, true);
    pilot.move_by((12*sqrt(2)+EXTRA_SPACE*sqrt(2)), 2000, false);
    pilot.turn_to(270, 2000, true);
    pilot.move_by(24*3-2*EXTRA_SPACE, 2000, true);
    pilot.turn_to(315, 2000, true);
    pilot.move_by((12*sqrt(2)+EXTRA_SPACE*sqrt(2)), 2000, false);
    pilot.turn_to(270, 2000, true);
    loader_state = Pneumatic::EXTENDED;

    pilot.move_by(-20.7, 2000, true);
    score_then_store(2500);
    score_then_store(500);
    loader_state = Pneumatic::EXTENDED;
    pilot.move_by(33.7, 2000, true);
    delay_for(1500);
    pilot.move_by(-33.7, 2000, true);
    score_then_store(2500);
    score_then_store(500);
    knock_in(30);

    pilot.move_by(15, 2000, true, -1);
    pilot.turn_to(315, 1500, true);
    pilot.move_by(30, 2000, true, -1);
    pilot.turn_to(340, 1500, true);
    pilot.timed_drive(2000, 30, 20, false);
    loader_state = Pneumatic::EXTENDED;
    pilot.timed_drive(1500, 80, 0, false);
}
