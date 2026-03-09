#include "autons.hpp"
#include "chromatic/control/linear_motions.hpp"
#include "chromatic/core/helpers.hpp"
#include "chromatic/shorthands.hpp"
#include "config.hpp"
#include "lidar.hpp"
#include "pros/llemu.hpp"
#include "subsystems.hpp"

using namespace chromatic;
using Exit = MotionController::Exit;

void knock_in(double speed, MotionController &pilot) {
    pilot.move_by(8, 500, Exit::MONO, speed);
    pilot.move_by(-8, 500, Exit::MONO, speed);
}

// please provide the stuff to use
void drive_test(MotionController &pilot) {
    localizer->set_pose(Pose{});

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

void turn_test(MotionController &pilot) {
    localizer->set_pose(Pose{});

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

void circle_drive(MotionController &pilot) {
    localizer->set_pose(Pose{});

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

void tune(MotionController& pilot) {

    Pose initial{Vec{0, 0}, to_rad(0)};
    localizer->set_pose(initial);
    pilot.set_cache(initial);

    // pilot.move_by(24, 1500);
    // pilot.move_by(-24, 1500);

    pilot.turn_to(180, 1500);
    // pilot.turn_to(180, 1500);
    pilot.turn_to(90, 1500);
    pilot.turn_to(270, 1500);
    pilot.turn_to(0, 1500);

    // pilot.move_to(Pose{Vec{24, 48}, 0});
    // pilot.move_to(Pose{Vec{0, 0}, 0}, FACE::BACK);

}

void left_both(MotionController &pilot) {
    Pose initial{Vec{-55.19, 19.1}, to_rad(0)};
    localizer->set_pose(initial);
    pilot.set_cache(initial);

    ms start = now();
    pilot.move_to(Pose{Vec{-36, 19.1},0},FACE::FWD, 800, -1, Exit::LOOSE);
    pilot.swing_to(25, DIR::LEFT, 750, Exit::MONO);
    body_state = Body::I_STORAGE;
    loader_state = Pneumatic::EXTENDED;
    pilot.move_by(12, 700, Exit::MONO);
    pilot.move_by(-3, 500, Exit::MONO, -1, DRIVE_TO_CCW);
    body_state = Body::DOWN_TOP;
    pilot.turn_to(135, 600, Exit::LOOSE, false, TURN_TO_BACK);
    body_state = Body::NOTHING;
    pilot.move_by(-19.5, 700);
    body_state = Body::S_MIDDLE;
    delay_for(350);
    body_state = Body::PREP_SCORE;
    pilot.move_to(Pose{Vec{-48, 48}, to_rad(135)}, FACE::FWD, 1300);
    body_state = Body::DOWN_TOP;
    pilot.turn_to(180, 800, Exit::MONO);
    // side_reset(Quad::Q2);
    body_state = Body::I_STORAGE;

    pilot.move_by(14, 900);
    pilot.move_by(-34, 1000);
    body_state = Body::M_REFRESH;
    delay_for(350);
    body_state = Body::S_FULL;
    delay_for(1700);

    loader_state = Pneumatic::RETRACTED;
    body_state = Body::NOTHING;
    pilot.swing_to(260, DIR::LEFT, 700, Exit::MONO);
    pilot.swing_to(185, DIR::RIGHT, 700, Exit::MONO);
    hook_state = Pneumatic::RETRACTED;
    pilot.refresh_cache();
    pilot.move_by(-20, 1000, Exit::LOOSE);

    pros::lcd::print(2, "route took %d ms", now()-start);

    delay_until(&start, 14000);
    pilot.turn_to(160);
    pilot.override_brake(true);

}

void left_rush(MotionController &pilot) {
    Pose initial{Vec{-55.19, 19.1}, to_rad(0)};
    localizer->set_pose(initial);
    pilot.set_cache(initial);

    ms start = now();
    pilot.move_to(Pose{Vec{-36, 19.1},0},FACE::FWD, 1000, -1, Exit::LOOSE, DRIVE_TO_CCW);
    pilot.swing_to(25, DIR::LEFT, 750, Exit::MONO);
    body_state = Body::I_STORAGE;
    loader_state = Pneumatic::EXTENDED;
    pilot.move_to(Pose{Vec{-48, 48}, to_rad(135)}, FACE::FWD, 1300);
    body_state = Body::DOWN_TOP;
    pilot.turn_to(180, 800, Exit::MONO);
    // side_reset(Quad::Q2);
    body_state = Body::I_STORAGE;

    pilot.move_by(14, 800);
    pilot.move_by(-34, 1000);
    body_state = Body::M_REFRESH;
    delay_for(350);
    body_state = Body::S_FULL;
    delay_for(1500);

    loader_state = Pneumatic::RETRACTED;
    body_state = Body::NOTHING;
    pilot.swing_to(260, DIR::LEFT, 700, Exit::MONO);
    pilot.swing_to(185, DIR::RIGHT, 700, Exit::MONO);
    hook_state = Pneumatic::RETRACTED;
    pilot.refresh_cache();
    pilot.move_by(-20, 1000, Exit::LOOSE);
    pilot.turn_to(160);

    pros::lcd::print(2, "route took %d ms", now()-start);

    delay_until(&start, 14000);
    pilot.override_brake(true);

}

void right_both(MotionController &pilot) {
    Pose initial{Vec{-55.19, -16.56}, to_rad(0)};
    localizer->set_pose(initial);
    pilot.set_cache(initial);

    const double imove_dx = 19.5 + 14.5 * cos(PI/6);
    const double imove_dy = 14.5 * sin(PI/6);

    double imove_L = 21;
    double imove_angle = to_deg(std::atan2(imove_dy, imove_dx - imove_L));
    double imove_reset = imove_dy / sin(to_rad(imove_angle));

    ms start = now();


    body_state = Body::I_STORAGE;

    pilot.move_by(imove_L, 800, Exit::LOOSE, -1);
    pilot.turn_to(360-imove_angle, 600, Exit::LOOSE, false);
    loader_state = Pneumatic::EXTENDED;
    pilot.move_by(imove_reset, 800, Exit::LOOSE, 40);
    Pose tgt = pilot.cache.get_pose();
    pros::lcd::print(4, "x: %.2f, y: %.2f, h:%.2f", tgt.pos.x, tgt.pos.y, to_deg(tgt.dir));


    // Pose sum{Vec{-23.13, -23.81}, to_rad(326.75)};
    // pilot.move_to(sum, FACE::FWD, 2000, 50);
    // loader_state = Pneumatic::EXTENDED;



    // loader_state = Pneumatic::RETRACTED;

    body_state = Body::DOWN_TOP;
    pilot.turn_to(225, 800, Exit::LOOSE, false);
    pilot.move_by(25*sqrt(2), 1000, Exit::TIGHT, -1);
    pilot.turn_to(180, 800, Exit::LOOSE, false);
    body_state = Body::I_STORAGE;
    pilot.move_by(14, 1000);
    body_state = Body::NOTHING;
    pilot.move_by(-34, 1000);
    body_state = Body::M_REFRESH;
    delay_for(300);
    body_state = Body::S_FULL;
    loader_state = Pneumatic::RETRACTED;
    delay_for(1450);
    // body_state = Body::M_REFRESH;
    // delay_for(300);
    body_state = Body::NOTHING;
    pilot.swing_to(295, DIR::LEFT, 800, Exit::MONO);
    pilot.turn_to(185, 1000, Exit::LOOSE, false, TURN_TO_BACK);
    hook_state = Pneumatic::RETRACTED;
    pilot.move_by(-20, 1500, Exit::LOOSE);
    pros::lcd::print(2, "route took %d ms", now()-start);
    pilot.turn_to(160);
    pilot.override_brake(true);
}

void right_rush(MotionController &pilot) {
    bool do_low = false;
    Pose initial{Vec{-47, -7.2}, to_rad(270)};
    localizer->set_pose(initial);
    // side_reset(Quad::Q3);
    pilot.set_cache(initial);

    auto hooking = [&] {
        body_state = Body::NOTHING;
        pilot.swing_to(270, DIR::LEFT, 900, Exit::MONO);
        pilot.swing_to(195, DIR::RIGHT, 800, Exit::MONO);
        hook_state = Pneumatic::RETRACTED;
        pilot.move_by(-25, 800, Exit::LOOSE);
    };

    pilot.move_by(32.8, 1000);
    loader_state = Pneumatic::EXTENDED;
    pilot.swing_to(180, DIR::RIGHT, 750);

    Vec ideal = pilot.cache.get_pos();
    // side_reset(Quad::Q3);
    // delay_for(50);
    // pros::lcd::print(6, "post x %.2f, y %.2f", localizer->get_pose().pos.x, localizer->get_pose().pos.y);
    // return;
    body_state = Body::I_STORAGE;
    pilot.move_by(7, 800);

    // delay in loader?

    pilot.move_by(-34, 1000);
    loader_state = Pneumatic::RETRACTED;
    body_state = Body::M_REFRESH;
    delay_for(200);
    body_state = Body::S_FULL;
    delay_for(1500);
    body_state = Body::I_STORAGE;

    // pilot.move_by(5, 1000, )

    if (do_low) {
        pilot.move_to(Pose{Vec{-24, -24}, to_rad(45)},FACE::FWD, 2000, 40);
        pilot.move_by(17, 1000);
        body_state = Body::S_LOW;
        delay_for(1500);
        body_state = Body::NOTHING;
        pilot.move_to(Pose{Vec{-36, -12}, 0}, FACE::BACK, 2000);
        hook_state = Pneumatic::RETRACTED;
        pilot.move_by(24);
    } else {
        hooking();
    }
    pilot.override_brake(true);
}

void solo(MotionController &pilot) {
    Pose initial{Vec{-47, -7.2}, to_rad(270)};
    localizer->set_pose(initial);
    pilot.set_cache(initial);

    pilot.move_by(32.8, 1000);
    loader_state = Pneumatic::EXTENDED;
    pilot.swing_to(180, DIR::RIGHT, 750);

    body_state = Body::I_STORAGE;
    pilot.move_by(7, 800);


    pilot.move_by(-34, 1000);
    loader_state = Pneumatic::RETRACTED;
    body_state = Body::M_REFRESH;
    delay_for(350);
    body_state = Body::S_FULL;
    delay_for(1500);
    body_state = Body::I_STORAGE;

}


void old_skills(MotionController &pilot) {
    Pose initial{Vec{-48, -16.8}, to_rad(270)};
    localizer->set_pose(initial);
    pilot.set_cache(initial);

    const double EXTRA_SPACE = 1.8;

    auto store_state = [&] {body_state = Body::PREP_SCORE;};
    auto hold_state = [&] {body_state = Body::NOTHING;};

    auto score_then_store = [&](ms dur = 3500) {
        body_state = Body::M_REFRESH;
        delay_for(200);
        body_state = Body::S_FULL;
        delay_for(dur);
        body_state = Body::PREP_SCORE;
    };

    pilot.move_by(31, 1000, Exit::MONO, -1, DRIVE_TO_CW);

    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(180, 1000, Exit::MONO, false, TURN_TO_FWD);
    store_state();
    pilot.move_by(11.5, 1000, Exit::LOOSE);
    delay_for(1200); // previously 1500
    pilot.move_by(-10.5, 1000, Exit::MONO);
    loader_state = Pneumatic::RETRACTED;

    pilot.turn_to(315, 1000, Exit::LOOSE, false, TURN_TO_FWD);
    pilot.move_by((12*sqrt(2)+EXTRA_SPACE*sqrt(2)), 2000, Exit::LOOSE, -1, DRIVE_TO_CCW);
    pilot.turn_to(0, 1000, Exit::LOOSE, false);
    pilot.move_by(24*3-2*EXTRA_SPACE, 2000, Exit::MONO, -1, DRIVE_TO_CCW);
    pilot.turn_to(45, 1000, Exit::LOOSE, false);
    pilot.move_by((12*sqrt(2)+EXTRA_SPACE*sqrt(2)), 2000, Exit::TIGHT);
    pilot.turn_to(0, 1000, Exit::LOOSE, false, TURN_TO_BACK);

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
    knock_in(20, pilot);

    pilot.move_by(16, 2000, Exit::MONO, -1, DRIVE_TO_CCW);
    pilot.turn_to(90, 1000, Exit::MONO, false);
    pilot.move_by(24*4+1.5, 4000, Exit::MONO, -1, DRIVE_TO_CW);

    loader_state = Pneumatic::EXTENDED;
    pilot.turn_to(0, 1000, Exit::MONO, false, TURN_TO_FWD);
    store_state();
    pilot.move_by(19, 1000, Exit::LOOSE);
    delay_for(1200); // previously 1500
    pilot.move_by(-11, 2000, Exit::MONO);
    loader_state = Pneumatic::RETRACTED;

    pilot.turn_to(135, 1000, Exit::LOOSE, false, TURN_TO_FWD);
    pilot.move_by((12*sqrt(2)+EXTRA_SPACE*sqrt(2)), 2000, Exit::LOOSE, -1, DRIVE_TO_CCW);
    pilot.turn_to(180, 1000, Exit::LOOSE, false);
    pilot.move_by(24*3-2*EXTRA_SPACE, 2000, Exit::MONO, -1, DRIVE_TO_CCW);
    pilot.turn_to(225, 1000, Exit::LOOSE, false);
    pilot.move_by((12*sqrt(2)+EXTRA_SPACE*sqrt(2)), 2000, Exit::LOOSE);
    pilot.turn_to(180, 1000, Exit::LOOSE, false, TURN_TO_BACK);

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
    knock_in(20, pilot);

    pilot.move_by(15, 2000, Exit::MONO, -1);
    pilot.turn_to(270, 1500, Exit::MONO);
    pilot.move_by(48, 2000, Exit::MONO, -1);
    pilot.turn_to(180, 1500, Exit::MONO);
    body_state = Body::S_FULL;
    pilot.move_by(50, 2000);

    pilot.timed_drive(500, 60, 40, false);
    pilot.timed_drive(500, -20, 0, false);
    pilot.timed_drive(500, 60, 40, false);
    pilot.timed_drive(500, -20, 0, false);
    pilot.timed_drive(500, 60, 40, false);
    pilot.override_brake();
}


void skills(MotionController &pilot) {
    Pose initial{Vec{-walls.left+7.56, -16.8}, to_rad(270)};
    localizer->set_pose(initial);
    // side_reset(Quad::Q3);
    pilot.set_cache(initial);

    const double EXTRA_SPACE = 1.8;

    auto store_state = [&] {body_state = Body::PREP_SCORE;};
    auto hold_state = [&] {body_state = Body::NOTHING;};

    auto score_then_store = [&](ms dur = 3500) {
        body_state = Body::M_REFRESH;
        delay_for(200);
        body_state = Body::S_FULL;
        delay_for(dur);
        body_state = Body::DOWN_TOP;
    };

    { // clear + middle 1
        body_state = Body::I_STORAGE;
        pilot.timed_drive(1500, 35, 30);
        pilot.timed_drive(1500, 25, 0);
        loader_state = Pneumatic::EXTENDED;
        pilot.timed_drive(3000, -15, -30);
        loader_state = Pneumatic::RETRACTED;
        side_reset(Quad::Q2);
        Pose cp = localizer->get_pose();
        localizer->set_pose(Pose{Vec{cp.pos.x, 16}, to_rad(90)});
        pilot.swing_to(0, DIR::RIGHT, 1000, Exit::LOOSE);
        pilot.swing_to(90, DIR::LEFT, 1000, Exit::LOOSE);
        pilot.turn_to(0, 1000, Exit::LOOSE, true);
        pilot.move_by(22, 1000, Exit::LOOSE, 40);
        body_state = Body::NOTHING;

        pilot.turn_to(130, 1000, Exit::LOOSE);
        // return;
        pilot.move_by(-25, 1500, Exit::LOOSE);
        body_state = Body::S_MIDDLE;
        delay_for(5000);

        pilot.move_by(37*sqrt(2), 3000, Exit::LOOSE);

    }

    {
        pilot.turn_to(180, 1000, Exit::LOOSE);
        // fwd_reset(Quad::Q2);
        // side_reset(Quad::Q2);

        loader_state = Pneumatic::EXTENDED;
        pilot.move_by(16, 1000);
        pilot.move_by(-16, 1500);

        pilot.swing_to(90, DIR::LEFT, 800);
        pilot.swing_to(180, DIR::RIGHT, 1000);
    }
}

void new_skills(MotionController &pilot) {
    Pose initial{Vec{-47, -7.2}, to_rad(270)};
    localizer->set_pose(initial);
    pilot.set_cache(initial);

    const double EXTRA_SPACE = 1.8;

    auto score_batch = [&](ms dur = 3000) {
        body_state = Body::M_REFRESH;
        delay_for(300);
        body_state = Body::S_FULL;
        delay_for(dur);
        body_state = Body::I_STORAGE;
    };

    auto loader_down = [&] {loader_state = Pneumatic::EXTENDED;};
    auto loader_up = [&] {loader_state = Pneumatic::RETRACTED;};


    // L1
    body_state = Body::DOWN_TOP;
    pilot.move_by(32.8, 1000);
    loader_down();
    pilot.swing_to(180, DIR::RIGHT, 750);
    body_state = Body::I_STORAGE;

    // Passage 1
    pilot.move_by(6, 1000);
    delay_for(1500);
    pilot.move_by(-6, 750);
    body_state = Body::DOWN_TOP;
    loader_up();
    pilot.swing_to(90, DIR::LEFT, 800);
    pilot.swing_to(180, DIR::RIGHT, 800);
    pilot.move_by(-72, 2000);

    // L2
    pilot.swing_to(270, DIR::RIGHT, 800);
    pilot.swing_to(180, DIR::LEFT, 800);
    pilot.turn_to(0, 800);
    side_reset(Quad::Q4);
    // fwd_reset(Quad::Q4);
    // delay_for(150000);
    loader_down();
    pilot.move_to(Pose{Vec{28, -48.7}, 0}, FACE::BACK, 1250);
    pilot.turn_to(0, 1000);
    pilot.cache.set_pos(pilot.cache.get_pos());
    score_batch();
    pilot.move_by(32, 1500);
    delay_for(1000);
    pilot.move_by(-32, 1000);
    loader_up();
    score_batch();

    // Crossing
    pilot.move_by(8, 1000);
    pilot.turn_to(90, 1000);
    body_state = Body::I_STORAGE;
    pilot.move_to(Pose{Vec{38, 49}, to_rad(90)}, FACE::FWD, 3500);
    loader_down();
    pilot.turn_to(0, 1000);
    // side_reset(Quad::Q1);
    pilot.move_to(Pose{Vec{62, 49.5}, to_rad(0)}, FACE::FWD, 1000, 30);
    delay_for(2000);

    // L3
    pilot.move_by(-6, 750);
    body_state = Body::DOWN_TOP;
    loader_up();
    pilot.swing_to(270, DIR::LEFT, 800);
    pilot.swing_to(0, DIR::RIGHT, 800);
    pilot.move_by(-72, 2000);

    // L4
    pilot.swing_to(90, DIR::RIGHT, 800);
    pilot.swing_to(0, DIR::LEFT, 800);
    pilot.turn_to(180, 800);
    // side_reset(Quad::Q2);
    // fwd_reset(Quad::Q4);
    // delay_for(150000);
    loader_down();

    pilot.move_to(Pose{Vec{-28, 51}, 180}, FACE::BACK, 1250);
    pilot.turn_to(180, 1000);
    pilot.cache.set_pos(pilot.cache.get_pos());
    score_batch();
    pilot.move_by(32, 1500);
    delay_for(1000);
    pilot.move_by(-32, 1000);
    loader_up();
    score_batch();

    pilot.turn_to(135, 1000);
    pilot.move_by(50, 1250);
    pilot.swing_to(180, DIR::LEFT, 1000);
    body_state = Body::S_FULL;
    pilot.timed_drive(2000, 30, -20);
    loader_down();
    delay_for(3000);


    // below is new code
    // pilot.move_to(Pose{Vec{48, -36}, to_rad(45)}, FACE::FWD, 3000, 40);
    // pilot.move_by(5, 1000);
    // pilot.turn_to(45, 1000);
    // pilot.move_to(Pose{Vec{63, -24}, to_rad(90)}, FACE::FWD, 3000, 40);

    // pilot.timed_drive(2500, 35, -20);
    // loader_down();
    // pilot.timed_drive(1500, -20, 20);

    // localizer->override_y(15.5);
    // side_reset(Quad::Q1);

    // pilot.move_to(Pose{Vec{48, 48}, to_rad(135)}, FACE::FWD, 3500);
    // pilot.turn_to(0, 1000);
    // side_reset(Quad::Q1);

    // pilot.move_to(Pose{Vec{28, 48}, 0}, FACE::BACK, 1250);
    // pilot.refresh_cache();
    // loader_down();
    // score_batch();
    // pilot.move_by(34, 1500);
    // delay_for(1000);
}
