#include "autons.h"
#include "chromatic/control/linear_motions.hpp"
#include "chromatic/core/helpers.hpp"
#include "chromatic/shorthands.hpp"
#include "config.h"
#include "subsystems.h"

using namespace chromatic;
using Exit = MotionController::Exit;

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
    pilot.move_by(60, 3000, Exit::MONO, -1, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(270, 1000, Exit::MONO, false, {to_rad(30), to_rad(90), 40, 0});
    pilot.move_by(60, 3000, Exit::MONO, -1, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(180, 1000, Exit::MONO, false, {to_rad(30), to_rad(90), 40, 0});
    pilot.move_by(60, 3000, Exit::MONO, -1, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(90, 1000, Exit::MONO, false, {to_rad(30), to_rad(90), 40, 0});
    pilot.move_by(60, 3000, Exit::MONO, -1, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(0, 1000, Exit::MONO);
}

void left_both(EncodersIMU &odom, MotionController &pilot) {
    const MotionController::Chain TURN_TO_FWD = {to_rad(15), to_rad(50), 30, 0};
    const MotionController::Chain TURN_TO_BACK = {to_rad(15), to_rad(50), -50, 0};
    const MotionController::Chain DRIVE_TO_CW =  {5, 10, 0, to_rad(-90)}; // revert to ±30 if this is too much
    const MotionController::Chain DRIVE_TO_CCW =  {5, 10, 0, to_rad(90)};

    auto knock_in = [&](double speed) {
        pilot.move_by(10, 500, Exit::MONO, speed);
        pilot.move_by(-10, 500, Exit::MONO, speed);
    };

    const double imove_dx = 20.5 + 14.5 * cos(PI/6);
    const double imove_dy = 14.5 * sin(PI/6);

    double imove_L = 22.2;
    double imove_angle = to_deg(std::atan2(imove_dy, imove_dx - imove_L));
    double imove_reset = imove_dy / sin(to_rad(imove_angle));

    body_state = Body::I_STORAGE;
    pilot.move_by(imove_L, 2000, Exit::TIGHT, -1, DRIVE_TO_CCW);
    pilot.turn_to(imove_angle, 800, Exit::LOOSE, false, TURN_TO_FWD);
    loader_state = Pneumatic::EXTENDED;
    pilot.move_by(imove_reset, 2000, Exit::LOOSE, 40, DRIVE_TO_CCW);

    pilot.turn_to(135, 1000, Exit::TIGHT, false);
    body_state = Body::NOTHING;
    pilot.move_by(-18.2, 1000, Exit::LOOSE, 40);
    body_state = Body::S_MIDDLE;
    delay_for(250);
    loader_state = Pneumatic::RETRACTED;
    body_state = Body::I_STORAGE;
    pilot.move_by(18.2+25*sqrt(2), 2000, Exit::TIGHT, -1);
    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(180, 2000, Exit::LOOSE, false, TURN_TO_FWD);
    pilot.move_by(14, 1000);
    pilot.move_by(-34, 1000);
    body_state = Body::S_FULL;
    delay_for(1500);
    knock_in(50);
    body_state = Body::NOTHING;
    pilot.move_by(12, 1000, Exit::MONO, -1, DRIVE_TO_CW);
    pilot.turn_to(135, 1000, Exit::LOOSE, false, TURN_TO_BACK);
    pilot.move_by(-10.5*sqrt(2), 1000, Exit::MONO, -1, DRIVE_TO_CCW);
    pilot.turn_to(180, 1000, Exit::LOOSE, false, TURN_TO_BACK);
    hook_state = Pneumatic::RETRACTED;
    pilot.move_by(-15, 1000, Exit::LOOSE, -1, DRIVE_TO_CW);
    pilot.turn_to(150, 1000);
}

void right_both(EncodersIMU &odom, MotionController &pilot) {
    const MotionController::Chain TURN_TO_FWD = {to_rad(15), to_rad(50), 30, 0};
    const MotionController::Chain TURN_TO_BACK = {to_rad(15), to_rad(50), -50, 0};
    const MotionController::Chain DRIVE_TO_CW =  {5, 10, 0, to_rad(-90)}; // revert to ±30 if this is too much
    const MotionController::Chain DRIVE_TO_CCW =  {5, 10, 0, to_rad(90)};


    auto knock_in = [&](double speed) {
        pilot.move_by(8, 500, Exit::MONO, speed);
        pilot.move_by(-8, 500, Exit::MONO, speed);
    };

    const double imove_dx = 20.5 + 14.5 * cos(PI/6);
    const double imove_dy = 14.5 * sin(PI/6);

    double imove_L = 22.2;
    double imove_angle = to_deg(std::atan2(imove_dy, imove_dx - imove_L));
    double imove_reset = imove_dy / sin(to_rad(imove_angle));

    body_state = Body::I_STORAGE;
    pilot.move_by(imove_L, 2000, Exit::TIGHT, -1, DRIVE_TO_CW);
    pilot.turn_to(360-imove_angle, 800, Exit::LOOSE, false, TURN_TO_FWD);
    loader_state = Pneumatic::EXTENDED;
    pilot.move_by(imove_reset, 2000, Exit::LOOSE, 40, DRIVE_TO_CW);
    loader_state = Pneumatic::RETRACTED;

    pilot.turn_to(45, 1000, Exit::TIGHT, false);
    body_state = Body::NOTHING;
    pilot.move_by(18.2, 1000, Exit::LOOSE, 40);
    body_state = Body::S_LOW;
    delay_for(500);
    loader_state = Pneumatic::RETRACTED;
    body_state = Body::I_STORAGE;
    pilot.move_by(-18.2-25*sqrt(2), 2000, Exit::TIGHT, -1);
    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(180, 2000, Exit::LOOSE, false, TURN_TO_FWD);
    pilot.move_by(14, 1000);
    // delay_for(400);
    pilot.move_by(-34, 1000);
    body_state = Body::M_REFRESH;
    delay_for(300);
    body_state = Body::S_FULL;
    delay_for(1500);
    knock_in(40);
    body_state = Body::NOTHING;
    pilot.move_by(12, 1000, Exit::MONO, -1, DRIVE_TO_CW);
    pilot.turn_to(135, 1000, Exit::LOOSE, false, TURN_TO_BACK);
    pilot.move_by(-10.5*sqrt(2), 1000, Exit::MONO, -1, DRIVE_TO_CCW);
    pilot.turn_to(180, 1000, Exit::LOOSE, false, TURN_TO_BACK);
    hook_state = Pneumatic::RETRACTED;
    pilot.move_by(-15, 1000, Exit::LOOSE, -1, DRIVE_TO_CW);
    pilot.turn_to(150, 1000);
}
void right_rush(EncodersIMU &odom, MotionController &pilot) {
    const MotionController::Chain TURN_TO_FWD = {to_rad(15), to_rad(50), 30, 0};
    const MotionController::Chain TURN_TO_BACK = {to_rad(15), to_rad(50), -50, 0};
    const MotionController::Chain DRIVE_TO_CW =  {5, 10, 0, to_rad(-90)}; // revert to ±30 if this is too much
    const MotionController::Chain DRIVE_TO_CCW =  {5, 10, 0, to_rad(90)};


    auto knock_in = [&](double speed) {
        pilot.move_by(8, 500, Exit::MONO, speed);
        pilot.move_by(-8, 500, Exit::MONO, speed);
    };

    const double imove_dx = 20.5 + 14.5 * cos(PI/6);
    const double imove_dy = 14.5 * sin(PI/6);

    double imove_L = 22.2;
    double imove_angle = to_deg(std::atan2(imove_dy, imove_dx - imove_L));
    double imove_reset = imove_dy / sin(to_rad(imove_angle));

    body_state = Body::I_STORAGE;
    pilot.move_by(imove_L, 2000, Exit::TIGHT, -1, DRIVE_TO_CW);
    pilot.turn_to(360-imove_angle, 800, Exit::LOOSE, false, TURN_TO_FWD);
    loader_state = Pneumatic::EXTENDED;
    pilot.move_by(imove_reset, 2000, Exit::LOOSE, 40, DRIVE_TO_CW);
    loader_state = Pneumatic::RETRACTED;

    pilot.turn_to(225, 1000, Exit::LOOSE, false, TURN_TO_FWD);
    loader_state = Pneumatic::EXTENDED;
    pilot.move_by(25*sqrt(2), 2000, Exit::TIGHT, -1);
    pilot.turn_to(180, 2000, Exit::LOOSE, false, TURN_TO_FWD);
    pilot.move_by(14, 1000);
    // delay_for(400);
    pilot.move_by(-34, 1000);
    body_state = Body::M_REFRESH;
    delay_for(300);
    body_state = Body::S_FULL;
    delay_for(1500);
    knock_in(40);
    body_state = Body::NOTHING;
    pilot.move_by(12, 1000, Exit::MONO, -1, DRIVE_TO_CW);
    pilot.turn_to(135, 1000, Exit::LOOSE, false, TURN_TO_BACK);
    pilot.move_by(-10.5*sqrt(2), 1000, Exit::MONO, -1, DRIVE_TO_CCW);
    pilot.turn_to(180, 1000, Exit::LOOSE, false, TURN_TO_BACK);
    hook_state = Pneumatic::RETRACTED;
    pilot.move_by(-15, 1000, Exit::LOOSE, -1, DRIVE_TO_CW);
    pilot.turn_to(150, 1000);
}

void solo(EncodersIMU &odom, MotionController &pilot) {

    pilot.move_by(31, 1000, Exit::MONO, -1,
        {5, 10, 0, to_rad(-30)}
    );
    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(270, 1000, Exit::MONO, false,
        {to_rad(15), to_rad(50), 40, 0}
    );
    body_state = Body::PREP_SCORE;
    pilot.move_by(10.6, 800, Exit::MONO);
    pilot.move_by(-33.1, 1500, Exit::MONO);
    body_state = Body::S_FULL;
    delay_for(1200);
    loader_state = Pneumatic::RETRACTED;
    body_state = Body::PREP_SCORE;
    pilot.turn_to(170, 1000, Exit::MONO, true);
    pilot.move_by(9.5, 1000, Exit::MONO);
    pilot.turn_to(180, 1000, Exit::MONO);
    pilot.move_by(48+6, 1800, Exit::MONO);
    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(225, 600, Exit::MONO);
    pilot.move_by(-17, 800, Exit::MONO);
    body_state = Body::S_MIDDLE;
    delay_for(600);
    body_state = Body::I_STORAGE;
    pilot.move_by(54.3, 1500, Exit::MONO);
    pilot.turn_to(270, 600, Exit::MONO);
    pilot.move_by(-22.5, 1200, Exit::MONO);
    body_state = Body::S_LOW;
    delay_for(300);
    body_state = Body::S_FULL;
    delay_for(1500);
}

void skills(EncodersIMU &odom, MotionController &pilot) {

    const double EXTRA_SPACE = 1.5;

    auto store_state = [&] {body_state = Body::PREP_SCORE;};
    auto hold_state = [&] {body_state = Body::NOTHING;};

    auto score_then_store = [&](ms dur = 3500) {
        body_state = Body::M_REFRESH;
        delay_for(350);
        body_state = Body::S_FULL;
        delay_for(dur);
        body_state = Body::PREP_SCORE;
    };

    auto knock_in = [&](double speed) {
        pilot.move_by(10, 500, Exit::MONO, speed);
        pilot.move_by(-10, 500, Exit::MONO, speed);
    };

    const MotionController::Chain TURN_TO_FWD = {to_rad(15), to_rad(50), 50, 0};
    const MotionController::Chain TURN_TO_BACK = {to_rad(15), to_rad(50), -50, 0};
    const MotionController::Chain DRIVE_TO_CW =  {5, 10, 0, to_rad(-60)};
    const MotionController::Chain DRIVE_TO_CCW =  {5, 10, 0, to_rad(60)};

    pilot.move_by(31, 1000, Exit::MONO, -1, DRIVE_TO_CW);

    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(270, 1000, Exit::MONO, false, TURN_TO_FWD);
    store_state();
    pilot.move_by(11.5, 1000, Exit::LOOSE);
    delay_for(1200); // previously 1500
    pilot.move_by(-10.5, 1000, Exit::MONO);
    loader_state = Pneumatic::RETRACTED;

    pilot.turn_to(45, 2000, Exit::MONO, false, TURN_TO_FWD);
    pilot.move_by((12*sqrt(2)+EXTRA_SPACE*sqrt(2)), 2000, Exit::LOOSE, -1, DRIVE_TO_CCW);
    pilot.turn_to(90, 2000, Exit::MONO, false, TURN_TO_FWD);
    pilot.move_by(24*3-2*EXTRA_SPACE, 2000, Exit::MONO, -1, DRIVE_TO_CCW);
    pilot.turn_to(135, 2000, Exit::MONO, false, TURN_TO_FWD);
    pilot.move_by((12*sqrt(2)+EXTRA_SPACE*sqrt(2)), 2000, Exit::LOOSE);
    pilot.turn_to(90, 2000, Exit::MONO, false, TURN_TO_BACK);

    loader_state = Pneumatic::EXTENDED;
    pilot.move_by(-20.2, 1000, Exit::MONO);
    score_then_store(1500);
    score_then_store(1300);
    pilot.move_by(34.5, 2000, Exit::LOOSE);
    delay_for(1000);
    pilot.move_by(-34.5, 2000, Exit::MONO);
    score_then_store(1500);
    score_then_store(1300);
    loader_state = Pneumatic::RETRACTED;
    knock_in(20);

    pilot.move_by(16, 2000, Exit::MONO, -1, DRIVE_TO_CCW);
    pilot.turn_to(180, 1000, Exit::MONO, false);
    pilot.move_by(24*4+1.5, 4000, Exit::MONO, -1, DRIVE_TO_CW);

    loader_state = Pneumatic::EXTENDED;
    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(90, 1000, Exit::MONO, false, TURN_TO_FWD);
    store_state();
    pilot.move_by(19, 1000, Exit::LOOSE);
    delay_for(1200); // previously 1500
    pilot.move_by(-11, 2000, Exit::MONO);
    loader_state = Pneumatic::RETRACTED;

    pilot.turn_to(225, 2000, Exit::MONO, false, TURN_TO_FWD);
    pilot.move_by((12*sqrt(2)+EXTRA_SPACE*sqrt(2)), 2000, Exit::LOOSE, -1, DRIVE_TO_CCW);
    pilot.turn_to(270, 2000, Exit::MONO, false, TURN_TO_FWD);
    pilot.move_by(24*3-2*EXTRA_SPACE, 2000, Exit::MONO, -1, DRIVE_TO_CCW);
    pilot.turn_to(315, 2000, Exit::MONO, false, TURN_TO_FWD);
    pilot.move_by((12*sqrt(2)+EXTRA_SPACE*sqrt(2)), 2000, Exit::LOOSE, -1);
    pilot.turn_to(270, 2000, Exit::MONO, false, TURN_TO_BACK);

    loader_state = Pneumatic::EXTENDED;
    pilot.move_by(-20.7, 1000, Exit::MONO);
    score_then_store(1500);
    score_then_store(1300);
    pilot.move_by(34.5, 2000, Exit::LOOSE);
    delay_for(1500);
    pilot.move_by(-34.5, 2000, Exit::MONO);
    score_then_store(1500);
    score_then_store(1300);
    loader_state = Pneumatic::RETRACTED;
    knock_in(20);

    pilot.move_by(15, 2000, Exit::MONO, -1);
    pilot.turn_to(0, 1500, Exit::MONO);
    pilot.move_by(48, 2000, Exit::MONO, -1);
    pilot.turn_to(270, 1500, Exit::MONO);
    body_state = Body::S_FULL;
    pilot.move_by(50, 2000);
    pilot.timed_drive(3000, 60, 0, false);

    // pilot.move_by(15, 2000, Exit::MONO, -1);
    // pilot.turn_to(315, 1500, Exit::MONO);
    // pilot.move_by(30, 2000, Exit::MONO, -1);
    // pilot.turn_to(340, 1500, Exit::MONO);
    // pilot.timed_drive(2000, 30, 20, false);
    // loader_state = Pneumatic::EXTENDED;
    // pilot.timed_drive(1500, 80, 0, false);
}
