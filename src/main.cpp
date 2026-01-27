#include "main.h"
#include "autons.h"
#include "chromatic/shorthands.hpp"
#include "config.h"
#include "pros/llemu.hpp"
#include "pros/screen.hpp"
#include "subsystems.h"

using namespace chromatic;

enum autons {SOLO, LEFT, RIGHT, SKILLS, DRIVE, TURN, CIRCLE, TUNE} auton_select{SOLO};

void initialize() {
	pros::lcd::initialize();

	left_mg.set_brake_mode_all(COAST);
	right_mg.set_brake_mode_all(COAST);
	storage.set_brake_mode(BRAKE);
	intake.set_brake_mode(BRAKE);
	outtake.set_brake_mode(BRAKE);

	hook.retract();
	loader.retract();

	odometry.calibrate();
	odometry.set_posev(PoseV{});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    comp_state = CompState::AUTON;
    pros::Task body_task([&]{run_body();});
    pros::Task odom_task([&]{odometry.localize();});

    odometry.set_pose(Pose{{0, 0}, 0});

    chassis.refresh_cache();
    chassis.set_pollrate(auton_pollrate);

    auton_select = TUNE;

    switch (auton_select) {
        case LEFT: left_both(odometry, chassis); break;
        case SOLO: solo_double(odometry, chassis); break;
        case SKILLS: skills(odometry, chassis); break;
        case DRIVE: drive_test(odometry, chassis); break;
        case TURN: turn_test(odometry, chassis); break;
        case CIRCLE: circle_drive(odometry, chassis); break;
        case TUNE: drive_test(odometry, chassis); /*turn_test(odometry, chassis); */break;
        default: break;
    }


    set_body(0, 0, 0);

    delay_for(60000);
    odometry.stop_loop();
    comp_state = CompState::REST;

    body_task.join();
    odom_task.join();
}

void opcontrol() {
    if (comp_state != CompState::REST) set_body(0, 0, 0);
    chassis.interrupt();

    comp_state = CompState::OPCONTROL;

    pros::Task odom_task([&]{odometry.localize();});

    odometry.set_pose(Pose({0, 0}, 0));

	while (true) {

		int fwd = master.get_analog(LY);
		int turn = master.get_analog(RX);

		// master.print(0, 0, "body: %d", static_cast<int>(body_state.load()));

		// fwd = expcurve(fwd, DRIVE_CURVE, 127);
		// turn = expcurve(turn, DRIVE_CURVE, 127);

		if (abs(fwd)+abs(turn) != 0) {
		    chassis.override_arcade(fwd, turn);
		} else {
		    chassis.override_brake();
		}

		damp_out = master.get_digital(BX);

		/*
		// Tiger Controls
		if (master.get_digital_new_press(AD)) hook_state = (hook_state == Pneumatic::EXTENDED ? Pneumatic::RETRACTED : Pneumatic::EXTENDED);
		if (master.get_digital_new_press(BA)) loader_state = (loader_state == Pneumatic::EXTENDED ? Pneumatic::RETRACTED : Pneumatic::EXTENDED);


		if (master.get_digital(R1)) {body_state = Body::I_STORAGE;}
		else if (master.get_digital(L2)) {body_state = Body::S_MIDDLE;}
		// else if (master.get_digital(R1)) {body_state = Body::S_LOW;}
		else if (master.get_digital(R2)) {body_state = Body::S_FULL;}
		else if (master.get_digital(L1)) {body_state = Body::E_FULL;}
		else {body_state = Body::NOTHING;}
		*/

		// Minyuan Controls
		if (master.get_digital_new_press(BB)) hook_state = (hook_state == Pneumatic::EXTENDED ? Pneumatic::RETRACTED : Pneumatic::EXTENDED);
		if (master.get_digital_new_press(BA)) loader_state = (loader_state == Pneumatic::EXTENDED ? Pneumatic::RETRACTED : Pneumatic::EXTENDED);

		if (master.get_digital(R2)) {body_state = Body::I_STORAGE;}
		else if (master.get_digital(AD)) {body_state = Body::S_MIDDLE;}
		else if (master.get_digital(R1)) {body_state = Body::S_LOW;}
		else if (master.get_digital(L2)) {body_state = Body::S_FULL;}
		else if (master.get_digital(L1)) {body_state = Body::E_FULL;}
		else {body_state = Body::NOTHING;}
        update_body();

		delay_for(OP_POLL_RATE);
	}

	comp_state = CompState::REST;

	set_body(0, 0, 0);
	odometry.stop_loop();
	odom_task.join();
}
