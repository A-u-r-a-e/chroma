#include "autons.h"
#include "chromatic/shorthands.hpp"
#include "pros/llemu.hpp"
#include "subsystems.h"

using namespace chromatic;

// please provide the stuff to use
void drive_test(EncodersIMU &odom, MotionController &pilot) {
    ms prev = now();
    double one = pilot.move_by(72, 3000, -1);
    pros::lcd::print(5, "1: %dms, %finch", now() - prev, one);
    prev = now();
    double two = pilot.move_by(-80, 3000, -1);
    pros::lcd::print(6, "2: %dms, %finch", now() - prev, two);
    prev = now();
    double three = pilot.move_by(8, 2000, -1);
    pros::lcd::print(7, "3: %dms, %finch", now() - prev, three);
}

void turn_test(EncodersIMU &odom, MotionController &pilot) {
    ms prev = now();
    double one = pilot.turn_to(to_rad(135), 4000, true, DIR::CCW);
    pros::lcd::print(3, "1: %dms, %fdeg", now() - prev, to_deg(one));
    // delay_for(1000);
    prev = now();
    double two = pilot.turn_to(to_rad(315), 4000, true, DIR::CCW);
    pros::lcd::print(4, "2: %dms, %fdeg", now() - prev, to_deg(two));
    // delay_for(1000);
    prev = now();
    double three = pilot.turn_to(to_rad(90), 4000, true, DIR::CW);
    pros::lcd::print(5, "3: %dms, %fdeg", now() - prev, to_deg(three));
    // delay_for(1000);
    prev = now();
    double four = pilot.turn_to(0, 4000, true);
    pros::lcd::print(6, "4: %dms, %fdeg", now() - prev, to_deg(four));
    prev = now();
}

void full_test(EncodersIMU &odom, MotionController &pilot) {
    // move_test(odom, pilot);
    drive_test(odom, pilot);
    turn_test(odom, pilot);
}

void red_left_side(EncodersIMU &odom, MotionController &pilot) {
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
    pilot.move_by(55, 2000, -1, true);
    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(to_rad(156), 750, true);
    pilot.move_by(7, 750, -1, true);
    delay_for(1000);
    pilot.move_by(-30, 2000, -1, true);
    body_state = Body::S_FULL;
    delay_for(1000);
    pilot.move_by(12, 2000, -1, true);
    pilot.turn_to(to_rad(111), 1000, true);
    pilot.move_by(-15, 2000, -1, true);
    pilot.turn_to(to_rad(156), 1000, true);
    hook_state = Pneumatic::RETRACTED;
    pilot.move_by(-18, 2000, -1, true);
}


void blue_left_side(EncodersIMU &odom, MotionController &pilot) {
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

void red_solo_awp(EncodersIMU &odom, MotionController &pilot) {
    // 48, 270º, 14, -31.7
    // 6.5, 155º, 28, 180º
    // 46, 220, -17
    // 52.5, 270º, 10, -31.7
    pilot.move_by(46.5, 2000, -1, false);
    pilot.turn_to(to_rad(270), 750, false);
    loader_state = Pneumatic::EXTENDED;
    body_state = Body::I_STORAGE;
    pilot.move_by(14, 1000, -1, true);
    delay_for(500);
    pilot.move_by(-30.5, 1500, -1, true);
    loader_state = Pneumatic::RETRACTED;
    body_state = Body::S_FULL;
    delay_for(1000);
    body_state = Body::I_STORAGE;

    // new low
    pilot.move_by(17, 800, -1, true);
    pilot.turn_to(to_rad(140), 750, true);
    pilot.move_by(26.5, 800, 40, true);
    pilot.move_by(22.5, 800, 40, true);
    body_state = Body::S_LOW;
    delay_for(750);
    body_state = Body::NOTHING;
    pilot.move_by(-15, 800, 40, true);
    // end low


    /*
    pilot.move_by(7, 600, -1, true);
    pilot.turn_to(to_rad(155), 750, true);
    pilot.move_by(28, 1000, 40, true);
    loader_state = Pneumatic::EXTENDED;
    delay_for(200);
    loader_state = Pneumatic::RETRACTED;
    // begin low
    pilot.turn_to(to_rad(125), 750, true);
    pilot.move_by(15, 800, 40, true);
    body_state = Body::S_LOW;
    delay_for(750);
    body_state = Body::NOTHING;
    pilot.move_by(-15, 800, -1, true);
    body_state = Body::I_STORAGE;
    // end low
    */
    double post = pilot.turn_to(to_rad(180), 600, true);
    pros::lcd::print(3,"error %f", to_deg(post));
    odom.set_pose(Pose{odom.get_pose().pos, odom.get_pose().dir - to_rad(8)});
    pilot.move_by(45, 1500, -1, true);
    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(to_rad(225), 750, true);
    loader_state = Pneumatic::RETRACTED;
    pilot.move_by(-16, 2000, 40, true); //revert to 16 for long goal route
    body_state = Body::S_MIDDLE;
    delay_for(1500);
    body_state = Body::I_STORAGE;


    pilot.move_by(46.5, 2000, -1, true);
    pilot.turn_to(to_rad(270), 750, true);
    // loader_state = Pneumatic::EXTENDED;
    // pilot.move_by(15, 2000, -1, true);
    // delay_for(750);
    // pilot.move_by(-30.7, 2000, -1, true);
    pilot.move_by(-15.7, 2000, -1, true);
    body_state = Body::S_FULL;
    delay_for(1500);
}


void red_solo_long(EncodersIMU &odom, MotionController &pilot) {
    auto smash_outtake = [&] (const ms downtime = 300) {
        task _actuation([&] {
            loader_state = Pneumatic::EXTENDED;
            delay_for(downtime);
            loader_state = Pneumatic::RETRACTED;
        });
    };

    // 48, 270º, 14, -31.7
    // 6.5, 155º, 28, 180º
    // 46, 220, -17
    // 52.5, 270º, 10, -31.7

    // Loader
    pilot.move_by(31, 900, -1, true);
    pilot.turn_to(to_rad(270), 600, true);
    loader_state = Pneumatic::EXTENDED;
    body_state = Body::PREP_SCORE;
    pilot.move_by(13, 800, -1, true);
    delay_for(250);

    // Long Goal 1
    pilot.move_by(-30.5, 800, -1, true);
    loader_state = Pneumatic::RETRACTED;
    body_state = Body::S_FULL;
    delay_for(1250);

    // Field Blocks 1
    body_state = Body::PREP_SCORE;
    pilot.turn_to(to_rad(160), 1000, true);
    pilot.move_by(10, 700, -1, true);
    // smash_outtake();
    // loader_state = Pneumatic::EXTENDED;
    loader_smashing = true;
    loader_unsmash_time = now() + 300;

    // Field Blocks 2
    pilot.turn_to(to_rad(187), 500, true);
    loader_state = Pneumatic::RETRACTED;
    pros::lcd::print(5, "%f", to_deg(odom.get_pose().dir));
    pilot.move_by(53.5, 1600, -1, true);
    // smash_outtake();
    // loader_state = Pneumatic::EXTENDED;
    loader_smashing = true;
    loader_unsmash_time = now() + 300;

    // Upper-Center Goal
    pilot.turn_to(to_rad(225), 650, true);
    pilot.move_by(-17, 1000, -1, true);
    body_state = Body::S_MIDDLE;
    delay_for(750);

    // Long goal
    body_state = Body::PREP_SCORE;
    pilot.move_by(52, 1300, -1, true);
    pilot.turn_to(to_rad(270), 500, true);
    pilot.move_by(-20, 800, -1, true);
    body_state = Body::M_REFRESH;
    delay_for(200);
    body_state = Body::S_FULL;
    delay_for(1500);
}


void blue_solo_awp(EncodersIMU &odom, MotionController &pilot) {
    // 48, 270º, 14, -31.7
    // 6.5, 155º, 28, 180º
    // 46, 220, -17
    // 52.5, 270º, 10, -31.7
    pilot.move_by(46, 2000, -1, true);
    pilot.turn_to(to_rad(270), 750, true);
    loader_state = Pneumatic::EXTENDED;
    body_state = Body::I_STORAGE;
    pilot.move_by(14, 1000, -1, true);
    delay_for(500);
    pilot.move_by(-30.5, 1500, -1, true);
    loader_state = Pneumatic::RETRACTED;
    body_state = Body::S_FULL;
    delay_for(1000);
    body_state = Body::I_STORAGE;
    pilot.move_by(7, 600, -1, true);
    pilot.turn_to(to_rad(155), 750, true);
    pilot.move_by(28, 1000, 40, true);
    loader_state = Pneumatic::EXTENDED;
    delay_for(200);
    loader_state = Pneumatic::RETRACTED;
    // begin low
    pilot.turn_to(to_rad(125), 750, true);
    pilot.move_by(15, 800, 40, true);
    body_state = Body::S_LOW;
    delay_for(750);
    body_state = Body::NOTHING;
    pilot.move_by(-15, 800, -1, true);
    body_state = Body::I_STORAGE;
    // end low
    double post = pilot.turn_to(to_rad(188), 600, true);
    pros::lcd::print(3,"error %f", to_deg(post));
    odom.set_pose(Pose{odom.get_pose().pos, odom.get_pose().dir - to_rad(8)});
    pilot.move_by(48, 1500, -1, true);
    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(to_rad(220), 750, true);
    loader_state = Pneumatic::RETRACTED;
    pilot.move_by(-20, 2000, 40, true); //revert to 16 for long goal route
    body_state = Body::S_MIDDLE;
    delay_for(1500);
    body_state = Body::I_STORAGE;
    pilot.move_by(46.5, 2000, -1, true);
    pilot.turn_to(to_rad(270), 750, true);
    // loader_state = Pneumatic::EXTENDED;
    // pilot.move_by(15, 2000, -1, true);
    // delay_for(750);
    // pilot.move_by(-30.7, 2000, -1, true);
    pilot.move_by(-15.7, 2000, -1, true);
    body_state = Body::S_FULL;
    delay_for(1500);
}

void skills(EncodersIMU &odom, MotionController &pilot) {}
