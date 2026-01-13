#include "subsystems.h"
#include "config.h"

using namespace chromatic;

std::atomic<CompState> comp_state{CompState::REST};
std::atomic<Body> body_state{Body::NOTHING};
std::atomic<Pneumatic> loader_state{Pneumatic::RETRACTED};
std::atomic<Pneumatic> hook_state{Pneumatic::EXTENDED};

void set_body(int cmd_intake, int cmd_storage, int cmd_outtake) {
    if (cmd_intake) intake.move(cmd_intake); else intake.brake();
    if (cmd_storage) storage.move(cmd_storage); else storage.brake();
    if (cmd_outtake) outtake.move(cmd_outtake); else outtake.brake();
}

void update_body() {
    int ballin = static_cast<int>(lidar.get_distance() < LIDAR_RANGE);
    switch (body_state.load()) {
    case Body::I_ONLY: set_body(127, 0, 0); break;
    case Body::I_STORAGE: set_body(127, ballin*40, ballin*4); break;
    case Body::E_FULL: set_body(-127, -127, -127); break;
    case Body::S_FULL: set_body(127, 127, 127); break;
    case Body::S_MIDDLE: set_body(127, 80, -127); break;
    case Body::S_LOW: set_body(-127, 0, 0); break;
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
