#include "subsystems.h"
#include "config.h"

using namespace chromatic;

std::atomic<CompState> comp_state = CompState::REST;
std::atomic<Body> body_state = Body::NOTHING;

std::atomic<int> intake_cmd(0);
std::atomic<int> outtake_cmd(0);
std::atomic<int> storage_cmd(0);
std::atomic<int> upper_outake(0);
std::atomic<int> scoring(0);
std::atomic<ms> filter_until(now());

void stop_robot() {
    intake_cmd.store(0);
    outtake_cmd.store(0);
    storage_cmd.store(0);
    upper_outake.store(0);
    scoring.store(0);

    intake.move(0);
    intake.brake();
    outtake.move(0);
    outtake.brake();
    storage.move(0);
    storage.brake();

    drivebase.brake(true);
}

void reset_update() {
    intake_cmd.store(0);
    outtake_cmd.store(0);
    storage_cmd.store(0);
    upper_outake.store(true);
    scoring.store(true);
}

void intake_storage_update(const double LIDAR_RANGE, bool slow) {
    intake_cmd.store((slow ? 100 : 127));
    if (lidar.get_distance() < LIDAR_RANGE) {
        storage_cmd.store((slow ? 40 : 40)); // if this is too much, switch to 70 (previous version)
        outtake_cmd.store(5);
    }
}

void middle_outtake_update() {
    intake_cmd.store(127);
    storage_cmd.store(80);
    outtake_cmd.store(-127);
    upper_outake.store(false);
    scoring.store(true);
}


void suck_update() {
    intake_cmd.store(127);
}

void spit_update() {
    intake_cmd.store(-127);
}

void full_linear_update(SIGN direction) {
    if (direction == SIGN::ZERO) return;

    intake_cmd.store(127 * static_cast<int>(direction));
    outtake_cmd.store(127 * static_cast<int>(direction));
    storage_cmd.store(127 * static_cast<int>(direction));
    scoring.store((static_cast<int>(direction) > 0));
}

void actuate_update() {
    double intkon = intake_cmd.load();
    double outkon = outtake_cmd.load();
    double storgon = storage_cmd.load();

    if (intkon == 0) intake.brake(); else intake.move(intkon);
    if (outkon == 0) outtake.brake(); else outtake.move(outkon);
    if (storgon == 0) storage.brake(); else storage.move(storgon);
}


void run_body(ms pollrate) {
    CompState current_state = comp_state;
    if (current_state == CompState::REST) return;
    while (comp_state == current_state) {
        reset_update();

        switch (body_state) {
            case Body::I_ONLY: suck_update(); break;
            case Body::I_STORAGE: intake_storage_update(LIDAR_RANGE, true); break;
            case Body::E_FULL: full_linear_update(SIGN::NEGATIVE); break;
            case Body::S_FULL: full_linear_update(SIGN::POSITIVE); break;
            case Body::S_MIDDLE: middle_outtake_update(); break;
            case Body::S_LOW: spit_update(); break;
            default: break;
        }

        actuate_update();
		delay_for(10);
    }
}

void stop_body() {
    comp_state = CompState::REST;
	reset_update();
	actuate_update();
}
