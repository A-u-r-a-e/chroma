#include "main.h"
#include "autons.h"
#include "config.h"
#include "pros/llemu.hpp"
#include "pros/screen.hpp"
#include "subsystems.h"

using namespace chromatic;

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
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    comp_state = CompState::AUTON;
    pros::Task body_task([&]{run_body();});
    pros::Task odom_task([&]{odometry.localize();});

    odometry.set_pose(Pose{{0, 0}, 0});

    chassis.set_pollrate(auton_pollrate);

    // turn_test(odometry, chassis);
    // drive_test(odometry, chassis);
    // move_test(odometry, chassis);
    // left_side(odometry, chassis);
    // full_test(odometry, chassis);
    solo_awp(odometry,chassis);

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
