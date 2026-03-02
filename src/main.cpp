#include "main.h"
#include "chromatic/shorthands.hpp"
#include "config.hpp"
#include "pros/llemu.hpp"
#include "subsystems.hpp"
#include <atomic>

using namespace chromatic;
void switch_auto(){
    auton_select = static_cast<autons>((auton_select + 1) % 7);
    master.print(0, 0, "Auton: %s", get_auton_name(auton_select));
};

void initialize() {
	pros::lcd::initialize();

	// hardware initialization
	left_mg.set_brake_mode_all(COAST);
	right_mg.set_brake_mode_all(COAST);
	storage.set_brake_mode(BRAKE);
	intake.set_brake_mode(BRAKE);
	outtake.set_brake_mode(BRAKE);

	hook.retract();
	loader.retract();

	// calibration
    localizer->calibrate();
    localizer->set_posev(PoseV{});

    // auton selection
    auton_select = RIGHT;
    master.clear();
    master.print(0, 0, "Auton: %s", get_auton_name(auton_select));
    if (master.get_digital_new_press(BY)) switch_auto();

    // tasks
    pros::Task body_task([&]{run_body();});
    pros::Task dresets_task([&]{run_dresets();});
    pros::Task odom_task([&]{localizer->localize(10);});


	master.rumble(".");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    comp_state = CompState::AUTON;
    master.rumble("-");

    chassis.refresh_cache();
    chassis.set_pollrate(auton_pollrate);

    switch (auton_select) {
        case LEFT: left_both(chassis); break;
        case RIGHT: right_both(chassis); break;
        case RIGHT_RUSH: right_rush(chassis); break;
        case SOLO: solo(chassis); break;
        case SKILLS: skills(chassis); break;
        case CIRCLE: circle_drive(chassis); break;
        case TUNE: /*drive_test(chassis);*/turn_test(chassis);break;
        default: break;
    }

    body_state = Body::NOTHING;

    comp_state = CompState::REST;
    master.rumble("-");
}

void opcontrol() {

    // autonomous();

    chassis.interrupt();

    master.rumble(".");
    comp_state = CompState::OPCONTROL;
    drivebase.reset_brake();

	while (true) {

		int fwd = master.get_analog(LY);
		int turn = master.get_analog(RX);

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
        // update_body();

		delay_for(OP_POLL_RATE);
	}

	comp_state = CompState::DISABLE;
	localizer->stop_loop();
	delay_for(50);
}
