#include "subsystems.h"
#include "config.h"

using namespace chromatic;

std::atomic<CompState> comp_state{CompState::REST};
std::atomic<Body> body_state{Body::NOTHING};
std::atomic<Pneumatic> loader_state{Pneumatic::RETRACTED};
std::atomic<Pneumatic> hook_state{Pneumatic::EXTENDED};

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

void update_body() {
    switch (body_state.load()) {
        case Body::I_ONLY: suck_update(); break;
        case Body::I_STORAGE: intake_storage_update(LIDAR_RANGE, true); break;
        case Body::E_FULL: full_linear_update(SIGN::NEGATIVE); break;
        case Body::S_FULL: full_linear_update(SIGN::POSITIVE); break;
        case Body::S_MIDDLE: middle_outtake_update(); break;
        case Body::S_LOW: spit_update(); break;
    }
    switch (hook_state.load()) {
        case Pneumatic::EXTENDED: if (!hook.is_extended()) {hook.extend();} break;
        case Pneumatic::RETRACTED: if (hook.is_extended()) {hook.retract();} break;
    }
    switch (loader_state.load()) {
        case Pneumatic::EXTENDED: if (!loader.is_extended()) {loader.extend();} break;
        case Pneumatic::RETRACTED: if (loader.is_extended()) {loader.retract();} break;
    }
}

void prepare_body() {
    intake_cmd.store(0);
    outtake_cmd.store(0);
    storage_cmd.store(0);
    upper_outake.store(true);
    scoring.store(true);
}

void actuate_body() {
    if (intake_cmd == 0) intake.brake(); else intake.move(intake_cmd);
    if (outtake_cmd == 0) outtake.brake(); else outtake.move(outtake_cmd);
    if (storage_cmd == 0) storage.brake(); else storage.move(storage_cmd);
}

void run_body(ms pollrate) {
    while (comp_state != CompState::REST) {
        prepare_body();
        update_body();
        actuate_body();
		delay_for(pollrate);
    }
}

void stop_body() {
    comp_state = CompState::REST;
    intake_cmd.store(0);
    outtake_cmd.store(0);
    storage_cmd.store(0);
    upper_outake.store(true);
    scoring.store(true);
    intake.brake();
    outtake.brake();
    storage.brake();
}
