#include "subsystems.h"
#include "chromatic/shorthands.hpp"
#include "config.h"

using namespace chromatic;

std::atomic<CompState> comp_state{CompState::REST};
std::atomic<Body> body_state{Body::NOTHING};
std::atomic<Pneumatic> loader_state{Pneumatic::RETRACTED};
std::atomic<Pneumatic> hook_state{Pneumatic::EXTENDED};

std::atomic<bool> loader_smashing{false};
std::atomic<ms> loader_unsmash_time{0};

void set_body(int cmd_intake, int cmd_storage, int cmd_outtake) {
    if (cmd_intake) intake.move(cmd_intake); else intake.brake();
    if (cmd_storage) storage.move(cmd_storage); else storage.brake();
    if (cmd_outtake) outtake.move(cmd_outtake); else outtake.brake();
}

void update_body() {
    int ballin = static_cast<int>(lidar.get_distance() < LIDAR_RANGE);
    bool update_top_switch = false;
    static ms last_top_switch = now();
    static int top_state = 1;

    // extra logic

    outtake.set_brake_mode(body_state == Body::PREP_SCORE ? HOLD : BRAKE);

    if (loader_smashing) {
        if (loader_unsmash_time < now()) {
            loader_smashing = false;
            loader_state = Pneumatic::RETRACTED;
        } else {
            loader_state = Pneumatic::EXTENDED;
        }
    }

    // actuation

    switch (body_state.load()) {
    case Body::I_ONLY: set_body(127, 0, 0); break;
    case Body::I_STORAGE: set_body(127, ballin*40, ballin*8); break;
    case Body::E_FULL: set_body(-127, -127, -127); break;
    case Body::S_FULL: set_body(127, 127, 127); break;
    case Body::S_MIDDLE: set_body(127, 100, -127); break;
    case Body::S_LOW: set_body(-127, 0, 0); break;
    case Body::M_REFRESH: set_body(-20, -127, -80); break;
    case Body::PREP_SCORE:
        set_body(127, 40, 0);
        update_top_switch = true;
        break;
    case Body::NOTHING: set_body(0, 0, 0);
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

void run_body(ms pollrate) {
    while (comp_state != CompState::REST) {
        update_body();
		delay_for(pollrate);
    }
}
