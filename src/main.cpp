#include "main.h"

using namespace chromatic;

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

    auton_select = SKILLS;

    master.print(0, 0, "Auton: %s", get_auton_name(auton_select));
    if (master.get_digital_new_press(BY)) {
        auton_select = static_cast<autons>((auton_select + 1) % 7);
        master.print(0, 0, "Auton: %s", get_auton_name(auton_select));
    }


	master.rumble(".");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    comp_state = CompState::AUTON;
    master.rumble("-");

    pros::Task body_task([&]{run_body();});
    pros::Task odom_task([&]{odometry.localize();});

    chassis.refresh_cache();
    chassis.set_pollrate(auton_pollrate);

    switch (auton_select) {
        case LEFT: left_both(odometry, chassis); break;
        case RIGHT: right_both(odometry, chassis); break;
        case RIGHT_RUSH: right_rush(odometry, chassis); break;
        case SOLO: solo(odometry, chassis); break;
        case SKILLS: skills(odometry, chassis); break;
        case CIRCLE: circle_drive(odometry, chassis); break;
        case TUNE: /*drive_test(odometry, chassis);*/turn_test(odometry, chassis);break;
        default: break;
    }


    body_state = Body::NOTHING;


    delay_for(60000);
    odometry.stop_loop();
    comp_state = CompState::REST;

    body_task.join();
    odom_task.join();
    master.rumble("-");
}

void opcontrol() {
    if (comp_state != CompState::REST) set_body(0, 0, 0);
    chassis.interrupt();

    master.rumble(".");
    comp_state = CompState::OPCONTROL;

    pros::Task odom_task([&]{odometry.localize();});

	while (true) {

		int fwd = master.get_analog(LY);
		int turn = master.get_analog(RX);

		if (abs(fwd)+abs(turn) != 0) {
		    chassis.override_arcade(fwd, turn);
		} else {
		    chassis.override_brake();
		}

		damp_out = master.get_digital(BX);

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
