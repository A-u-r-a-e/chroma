#include "autons.h"
#include "chromatic/shorthands.hpp"
#include "config.h"
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

void circle_drive(EncodersIMU &odom, MotionController &pilot) {
    hook_state = Pneumatic::RETRACTED;
    pilot.move_by(60, 3000, -1, true, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(to_rad(270), 1000, true, DIR::EITHER, {to_rad(30), to_rad(90), 40, 0});
    pilot.move_by(60, 3000, -1, true, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(to_rad(180), 1000, true, DIR::EITHER, {to_rad(30), to_rad(90), 40, 0});
    pilot.move_by(60, 3000, -1, true, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(to_rad(90), 1000, true, DIR::EITHER, {to_rad(30), to_rad(90), 40, 0});
    pilot.move_by(60, 3000, -1, true, {6, 20, 0, to_rad(-360)});
    pilot.turn_to(to_rad(0), 1000, true, DIR::EITHER);
}

void left_6_1(EncodersIMU &odom, MotionController &pilot) {

    // 19, 30º, 12
    loader_state = Pneumatic::RETRACTED;
    body_state = Body::I_STORAGE;
    pilot.move_by(19,  1000, -1, false);
    pilot.turn_to(to_rad(25), 1000, true, DIR::CW);
    // loader_state = Pneumatic::EXTENDED;
    pilot.move_by(11,  1500, 20, false);
    pilot.turn_to(to_rad(135), 1000, false, DIR::CW);

    pilot.move_by(-18, 1000, -1, true);
    // loader_state = Pneumatic::RETRACTED;
    body_state = Body::S_MIDDLE;
    delay_for(600);
    body_state = Body::I_STORAGE;
    pilot.turn_to(to_rad(125), 1000, false, DIR::EITHER);
    pilot.move_by(55.5, 2500, 60, true);

    pilot.turn_to(to_rad(180), 1000, false, DIR::CCW);
    loader_state = Pneumatic::EXTENDED;
    pilot.move_by(-9.5, 1000, -1, false);
    pilot.move_by(16.2, 1000, -1, true);
    delay_for(500);
    pilot.move_by(-30.5, 2000, -1, true);
    body_state = Body::S_LOW;
    delay_for(250);
    body_state = Body::S_FULL;
    delay_for(1500);
    pilot.move_by(12, 2000, -1, true);
    pilot.move_by(-24, 1000, -1, true);

    // pilot.turn_to(to_rad(135), 1000, true, DIR::EITHER, {to_rad(20), to_rad(10), -10, 0});
    // pilot.move_by(-10.5, 1000, -1, true);
    // pilot.turn_to(to_rad(180), 1000, true);
    // hook_state = Pneumatic::RETRACTED;
    // pilot.move_by(-12, 1000, -1, true);
    // pilot.turn_to(to_rad(135), 1000, true);

}

void left_7_0(EncodersIMU &odom, MotionController &pilot) {

    loader_state = Pneumatic::RETRACTED;
    body_state = Body::PREP_SCORE;
    pilot.move_by(19,  1000, 30, true, {5, 10, 0, to_rad(10)});
    pilot.turn_to(to_rad(30), 1000, true, DIR::CW, {to_rad(5), to_rad(5), 10, 0});
    loader_state = Pneumatic::EXTENDED;
    pilot.move_by(12,  1000, 30, true, {5, 10, 0, to_rad(60)});
    pilot.turn_to(to_rad(135), 1000, true, DIR::CW, {to_rad(5), to_rad(10), -10, 0});

    pilot.move_by(36.5, 2000, -1, true, {5, 10, 0, to_rad(10)});

    pilot.turn_to(to_rad(180), 750, true, DIR::CCW, {to_rad(20), to_rad(10), 10, 0});
    pilot.move_by(7, 1000, -1, true);
    delay_for(250);
    pilot.turn_to(to_rad(180), 750, true);
    pilot.move_by(-30, 2000, -1, true);
    body_state = Body::S_FULL;
    delay_for(1000);
    pilot.move_by(12, 2000, -1, true,{5, 20, 0, to_rad(-10)});
    pilot.turn_to(to_rad(135), 1000, true, DIR::EITHER, {to_rad(20), to_rad(10), -10, 0});
    pilot.move_by(-5, 2000, -1, true);
    pilot.turn_to(to_rad(180), 1000, true);
    hook_state = Pneumatic::RETRACTED;
    pilot.move_by(-18, 2000, -1, true);
}

void right_6_1(EncodersIMU &odom, MotionController &pilot) {

}

void right_7_0(EncodersIMU &odom, MotionController &pilot) {

}

void solo_single(EncodersIMU &odom, MotionController &pilot) {
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
    delay_for(1000);
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

void solo_double(EncodersIMU &odom, MotionController &pilot) {
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
    pilot.move_by(31.5, 900, -1, true);
    pilot.turn_to(to_rad(270), 600, true);
    loader_state = Pneumatic::EXTENDED;
    body_state = Body::I_STORAGE;
    pilot.move_by(12.5, 800, -1, true);
    delay_for(250);

    // Long Goal 1
    pilot.turn_to(to_rad(270), 600, true);
    pilot.move_by(-30, 1600, 20, true);
    loader_state = Pneumatic::RETRACTED;
    body_state = Body::S_LOW;
    delay_for(200);
    body_state = Body::S_FULL;
    delay_for(1750);

    // Field Blocks 1
    body_state = Body::PREP_SCORE;
    pilot.turn_to(to_rad(160), 1000, true);
    pilot.move_by(10, 700, -1, true);
    // smash_outtake();
    // loader_state = Pneumatic::EXTENDED;
    // loader_smashing = true;
    // loader_unsmash_time = now() + 300;

    // Field Blocks 2
    pilot.turn_to(to_rad(187), 500, true);
    loader_state = Pneumatic::RETRACTED;
    pros::lcd::print(5, "%f", to_deg(odom.get_pose().dir));
    pilot.move_by(53.7, 1600, -1, true);
    // smash_outtake();
    // loader_state = Pneumatic::EXTENDED;
    // loader_smashing = true;
    // loader_unsmash_time = now() + 300;

    // Upper-Center Goal
    pilot.turn_to(to_rad(225), 650, true);
    pilot.move_by(-18, 1000, -1, true);
    body_state = Body::S_MIDDLE;
    delay_for(700);

    // Long goal
    body_state = Body::I_STORAGE;
    pilot.move_by(48, 1300, -1, true);
    pilot.turn_to(to_rad(275), 500, true);
    /** copy paste */
    loader_state = Pneumatic::EXTENDED;
    body_state = Body::I_STORAGE;
    pilot.move_by(15, 800, -1, true);
    pilot.turn_to(to_rad(273), 600, true);
    delay_for(250);
    pilot.move_by(-30.5, 800, -1, true);
    loader_state = Pneumatic::RETRACTED;
    body_state = Body::S_LOW;
    delay_for(200);
    body_state = Body::S_FULL;
    delay_for(1250);

    // pilot.move_by(-20, 800, -1, true);
    // body_state = Body::M_REFRESH;
    // delay_for(200);
    // body_state = Body::S_FULL;
    // delay_for(1500);
}



void solo_autism(EncodersIMU &odom, MotionController &pilot) {
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
    pilot.move_by(31.5, 900, -1, true);
    pilot.turn_to(to_rad(270), 600, true);
    loader_state = Pneumatic::EXTENDED;
    body_state = Body::I_STORAGE;
    pilot.move_by(12.5, 800, -1, true);
    delay_for(250);

    // Long Goal 1
    pilot.turn_to(to_rad(270), 600, true);
    pilot.move_by(-30, 1600, 20, true);
    loader_state = Pneumatic::RETRACTED;
    body_state = Body::S_LOW;
    delay_for(200);
    body_state = Body::S_FULL;
    delay_for(1750);

    // Field Blocks 1
    body_state = Body::PREP_SCORE;
    pilot.turn_to(to_rad(160), 1000, true);
    pilot.move_by(10, 700, -1, true);
    // smash_outtake();
    // loader_state = Pneumatic::EXTENDED;
    // loader_smashing = true;
    // loader_unsmash_time = now() + 300;

    // Field Blocks 2
    pilot.turn_to(to_rad(187), 500, true);
    loader_state = Pneumatic::RETRACTED;
    pros::lcd::print(5, "%f", to_deg(odom.get_pose().dir));
    pilot.move_by(53.7, 1600, -1, true);
    // smash_outtake();
    // loader_state = Pneumatic::EXTENDED;
    // loader_smashing = true;
    // loader_unsmash_time = now() + 300;

    // Upper-Center Goal
    pilot.turn_to(to_rad(225), 650, true);
    pilot.move_by(-18, 1000, -1, true);
    body_state = Body::S_MIDDLE;
    delay_for(1000);

    // Long goal
    body_state = Body::I_STORAGE;
    pilot.move_by(48, 1300, -1, true);
    pilot.turn_to(to_rad(275), 500, true);
    /** copy paste */
    loader_state = Pneumatic::EXTENDED;
    body_state = Body::I_STORAGE;
    pilot.move_by(15, 800, -1, true);
    pilot.turn_to(to_rad(273), 600, true);
    delay_for(250);
    pilot.move_by(-30.5, 800, -1, true);
    loader_state = Pneumatic::RETRACTED;
    body_state = Body::S_LOW;
    delay_for(200);
    body_state = Body::S_FULL;
    delay_for(1250);

    // pilot.move_by(-20, 800, -1, true);
    // body_state = Body::M_REFRESH;
    // delay_for(200);
    // body_state = Body::S_FULL;
    // delay_for(1500);
}


void skills(EncodersIMU &odom, MotionController &pilot) {
    loader_state = Pneumatic::RETRACTED;
    hook_state = Pneumatic::EXTENDED;
    body_state = Body::I_STORAGE;
    odom.set_posev({});

    auto half_world = [&](bool flip = false) {
        double amt = flip ? 180 : 0;

        // first loader
        pilot.move_by(-16, 2000, -1, true);
        loader_state = Pneumatic::EXTENDED;
        body_state = Body::I_STORAGE;
        delay_for(500);
        pilot.move_by(28, 2000, -1, true);
        delay_for(1000);
        pilot.move_by(-12, 2000, -1, true);
        loader_state = Pneumatic::RETRACTED;

        // across the field
        pilot.turn_to(to_rad(225+amt), 1000, false);
        pilot.move_by(-16, 2000, -1, true);
        pilot.turn_to(to_rad(90+amt), 1000, false);
        pilot.move_by(72, 4000, -1, true);
        pilot.turn_to(to_rad(145+amt), 1000, false);
        pilot.move_by(12.5, 2000, -1, true);
        pilot.turn_to(to_rad(90+amt), 1000, false);

        // score set one
        loader_state = Pneumatic::EXTENDED;
        pilot.move_by(-20, 2000, -1, true);
        body_state = Body::S_LOW;
        delay_for(300);
        body_state = Body::S_FULL;
        delay_for(1500);
        body_state = Body::S_LOW;
        delay_for(300);
        body_state = Body::S_FULL;
        delay_for(1500);
        body_state = Body::I_STORAGE;
        // pilot.turn_to(to_rad(85+amt), 1000, true);

        // loader 2
        body_state = Body::I_STORAGE;
        pilot.move_by(30.5, 2000, -1, true);
        delay_for(1000);

        // score set two
        pilot.move_by(-30.5, 2000, -1, true);
        body_state = Body::S_LOW;
        delay_for(300);
        body_state = Body::S_FULL;
        delay_for(1500);
        body_state = Body::S_LOW;
        delay_for(300);
        body_state = Body::S_FULL;
        delay_for(1500);
        body_state = Body::I_STORAGE;
        pilot.move_by(18, 2000, -1, true);
        loader_state = Pneumatic::RETRACTED;
        pilot.turn_to(to_rad(95+amt), 1000, true);
    };

    auto mini_world = [&]{
        // first loader
        pilot.move_by(-16, 2000, -1, true);
        loader_state = Pneumatic::EXTENDED;
        body_state = Body::I_STORAGE;
        delay_for(500);
        pilot.move_by(28.5, 2000, -1, true);
        delay_for(1000);
        pilot.move_by(-30, 2000, -1, true);
        loader_state = Pneumatic::RETRACTED;
        body_state = Body::S_LOW;
        delay_for(300);
        body_state = Body::S_FULL;
        delay_for(3000);
        pilot.move_by(10, 2000, -1, true);
        pilot.override_arcade(-100, 0);
        delay_for(500);
        pilot.override_brake();
        pilot.move_by(16, 2000, -1, true);
        pilot.turn_to(to_rad(225), 1500, true);
        pilot.move_by(40, 2000, -1, true);
        pilot.turn_to(to_rad(200), 1500, true);
        pilot.override_arcade(100, 0);
        delay_for(2000);
        pilot.override_brake();


    };

    // to first pose
    pilot.move_by(31, 2000, -1, true);
    pilot.turn_to(to_rad(270), 1500, true);

    mini_world();
    return;



    half_world();

    pilot.move_by(-5, 2000, -1, true);

    // move to second side
    pilot.turn_to(to_rad(180), 1500, true);
    pilot.move_by(95, 2000, -1, true);
    pilot.turn_to(to_rad(90), 1500, true);

    pilot.move_by(5, 2000, -1, true);

    // reset
    loader_state = Pneumatic::RETRACTED;
    hook_state = Pneumatic::EXTENDED;
    body_state = Body::I_STORAGE;

    half_world(true);

    // clear park zone
    pilot.turn_to(to_rad(315), 1500, true);
    pilot.move_by(24, 3000, -1, true);
    pilot.turn_to(to_rad(350), 1500, true);
    pilot.move_by(52, 5000, 30, true);

}
