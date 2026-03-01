#include "main.h"
#include "chromatic/shorthands.hpp"
#include "config.hpp"
#include "pros/llemu.hpp"
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
    auton_select = LEFT;
    master.clear();
    master.print(0, 0, "Auton: %s", get_auton_name(auton_select));
    if (master.get_digital_new_press(BY)) switch_auto();

    // tasks
    pros::Task body_task([&]{run_body();});
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


    delay_for(60000);
    // localizer->stop_loop();
    comp_state = CompState::REST;
    master.rumble("-");
}

std::atomic<bool> ping{false};
void ping_loop() {
    ping = true;
    int tick = 0;
    while (ping) {
        tick = (tick+1)%25;

	    l_lfront.update();
		l_rfront.update();
		l_left.update();
		l_right.update();
		delay_for(10);

		if (tick==0) {
            double ping_lfront = l_lfront.get_denoised();
            double ping_rfront = l_rfront.get_denoised();
            master.print(0, 0, "Front %.2f %.2f", ping_lfront, ping_rfront);
            delay_for(20);
            double ping_left = l_left.get_denoised();
            double ping_right = l_right.get_denoised();
            master.print(1, 0, "Sides %.2f %.2f", ping_left, ping_right);
            delay_for(20);
		} else {
		    delay_for(40);
		}

    }
}

void opcontrol() {

    // autonomous();

    if (comp_state != CompState::REST) set_body(0, 0, 0);
    chassis.interrupt();

    master.rumble(".");
    comp_state = CompState::OPCONTROL;

    pros::Task pings(ping_loop);

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
        update_body();

		delay_for(OP_POLL_RATE);
	}

	comp_state = CompState::REST;

	set_body(0, 0, 0);
	localizer->stop_loop();
	ping = false;
	pings.join();
}
