#include "main.h"
#include "autons.h"
#include "chromatic/shorthands.hpp"
#include "config.h"
#include "pros/llemu.hpp"
#include "pros/screen.hpp"
#include "subsystems.h"

using namespace chromatic;

enum autons {RED_SOLO, RED_SOLO_LONG, BLUE_SOLO, BLUE_SOLO_LONG, RED_LEFT, RED_RIGHT, BLUE_LEFT, BLUE_RIGHT, SKILLS, TURNTEST, DRIVETEST} auton_select{RED_SOLO};

void initialize() {
	pros::lcd::initialize();

	left_mg.set_brake_mode_all(COAST);
	right_mg.set_brake_mode_all(COAST);
	storage.set_brake_mode(BRAKE);
	intake.set_brake_mode(BRAKE);
	outtake.set_brake_mode(BRAKE);

	hook.extend();
	loader.retract();

	odometry.calibrate();
	odometry.set_posev(PoseV{});

	// comp_state = CompState::REST;
	// while (comp_state == CompState::REST && !master.get_digital_new_press(BX)) {
	//     switch (auton_select) {
	// 	case RED_SOLO: master.print(0, 0, "RED SOLO"); break;
	// 	case BLUE_SOLO: master.print(0, 0, "BLUE SOLO"); break;
	// 	case RED_LEFT: master.print(0, 0, "RED LEFT"); break;
	// 	case RED_RIGHT: master.print(0, 0, "RED RIGHT"); break;
	// 	case BLUE_LEFT: master.print(0, 0, "BLUE LEFT"); break;
	// 	case BLUE_RIGHT: master.print(0, 0, "BLUE RIGHT"); break;
	// 	case TURNTEST: master.print(0, 0, "TURNTEST"); break;
	// 	case DRIVETEST: master.print(0, 0, "DRIVETEST"); break;
	// 	case SKILLS: master.print(0, 0, "SKILLS"); break;
	// 	}

	// 	if (master.get_digital_new_release(BA)) auton_select = static_cast<autons>((static_cast<int>(auton_select) + 1)%9);
	// }
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    comp_state = CompState::AUTON;
    pros::Task body_task([&]{run_body();});
    pros::Task odom_task([&]{odometry.localize();});

    odometry.set_pose(Pose{{0, 0}, 0});

    chassis.set_pollrate(auton_pollrate);

    auton_select = RED_SOLO_LONG;

    switch (auton_select) {
        case TURNTEST: turn_test(odometry, chassis); break;
        case DRIVETEST: drive_test(odometry, chassis); break;
        case RED_LEFT: red_left_side(odometry, chassis); break;
        case BLUE_LEFT: blue_left_side(odometry, chassis); break;
        case RED_SOLO: red_solo_awp(odometry, chassis); break;
        case BLUE_SOLO: blue_solo_awp(odometry, chassis); break;
        case RED_SOLO_LONG: red_solo_long(odometry, chassis); break;
        default: break;
    }

    // chassis.turn_to(to_rad(90), 4000, false);
    // chassis.turn_to(to_rad(0), 4000, false);
    delay_for(10000);

    set_body(0, 0, 0);
    odometry.stop_loop();

    body_task.join();
    odom_task.join();
    comp_state = CompState::REST;
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

		master.print(0, 0, "body: %d", static_cast<int>(body_state.load()));

		if (abs(fwd)+abs(turn) != 0) {
		    chassis.override_arcade(fwd, turn);
		} else {
		    chassis.override_brake();
		}

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
