#include "main.h"

using namespace chromatic;

void initialize() {
	pros::lcd::initialize();

	left_mg.set_brake_mode_all(COAST);
	right_mg.set_brake_mode_all(COAST);
	storage.set_brake_mode(BRAKE);
	intake.set_brake_mode(BRAKE);
	outtake.set_brake_mode(BRAKE);

	ear.extend();
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

    auton_chassis.set_pollrate(auton_pollrate);

    drive_test();

    stop_body();
    odometry.stop_loop();

    body_task.join();
    odom_task.join();
    comp_state = CompState::REST;
}

void opcontrol() {
    comp_state = CompState::OPCONTROL;
    master.print(0, 0, "helloooo");
    pros::lcd::print(1, "in opcontrol");
    delay_for(5000);
    auton_chassis.interrupt();
    stop_body();
    delay_for(50);

    pros::Task body_task([&]{run_body();});

	while (true) {

		int fwd = master.get_analog(LY);
		int turn = master.get_analog(RX);

		if (abs(fwd)+abs(turn) != 0) {
		    drivebase.arcade_drive(fwd, turn);
		} else {
		    drivebase.brake();
		}

		if (master.get_digital_new_press(BB)) ear.toggle();
		if (master.get_digital_new_press(BA)) loader.toggle();

		if (master.get_digital(R2)) body_state = Body::I_STORAGE;
		if (master.get_digital(AD)) body_state = Body::S_MIDDLE;
		if (master.get_digital(R1)) body_state = Body::S_LOW;
		if (master.get_digital(L2)) body_state = Body::S_FULL;
		if (master.get_digital(L1)) body_state = Body::E_FULL;

		delay_for(OP_POLL_RATE);
	}

	comp_state = CompState::REST;

	stop_body();
	body_task.join();
}
