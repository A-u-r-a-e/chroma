#include "main.h"
#include "chromatic/chassis/odometry.hpp"
#include "hardware.h"

using namespace chromatic;

void initialize() {
	pros::lcd::initialize();

	ms init_time = now();
	inertial.reset();
	pros::lcd::print(1, "IMU Calibrated in %dms!", now()-init_time);

}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {

	std::unique_ptr<Differential> chassis = std::make_unique<Differential>(
		left_mg, right_mg,
		2.75, 72.0 / 48.0, 11.92
	);

	while (true) {

		int fwd = master.get_analog(LY);
		int turn = master.get_analog(RX);

		if (abs(fwd)+abs(turn) == 0) {
		    chassis->arcade_drive(fwd, turn);
		} else {
		    chassis->brake();
		}

		delay_for(OP_POLL_RATE);
	}
}
