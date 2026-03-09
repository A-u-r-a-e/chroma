#include "subsystems.hpp"
#include "chromatic/core/helpers.hpp"
#include "chromatic/shorthands.hpp"
#include "config.hpp"

using namespace chromatic;

std::atomic<Body> body_state{Body::NOTHING};
std::atomic<Pneumatic> loader_state{Pneumatic::RETRACTED};
std::atomic<Pneumatic> hook_state{Pneumatic::EXTENDED};

void set_body(int cmd_intake, int cmd_storage, int cmd_outtake, int sus_outtake) {
    double volt_rpm = 200.0 / 127.0;
    outtake.set_brake_mode(sus_outtake==1 ? HOLD : sus_outtake == -1 ? COAST : BRAKE);
    if (cmd_intake) intake.move(cmd_intake); else intake.brake();
    if (cmd_storage) storage.move_velocity(cmd_storage * volt_rpm); else storage.brake();
    if (cmd_outtake) outtake.move(cmd_outtake); else outtake.brake();
}

void update_body() {
    // actuation

    int c_intake{0}, c_storage{0}, c_outtake{0}, sus_outtake{0};
    switch (body_state.load()) {
        case Body::I_ONLY:
            c_intake = 127;
            c_storage = 0;
            c_outtake = 0;
            break;
        case Body::SLOW_STORAGE:
            c_intake = 60;
            c_storage = 60;
            c_outtake = 0;
            break;
        case Body::E_FULL:
            c_intake = -127;
            c_storage = -127;
            c_outtake = -127;
            break;
        case Body::S_FULL:
            c_intake = 127;
            c_storage = 127;
            c_outtake = 127;
            break;
        case Body::S_MIDDLE:
            c_intake = 127;
            c_storage = 80;
            c_outtake = (auton_select == SKILLS ? -45 : -127);
            break;
        case Body::S_LOW:
            c_intake = (auton_select == SKILLS  ? -100 : -127);
            c_storage = -80;
            c_outtake = 0;
            break;
        case Body::M_REFRESH:
            c_intake = -35;
            c_storage = -80;
            c_outtake = 127;
            break;
        case Body::I_STORAGE:
            c_intake = 127;
            c_storage = 127;
            c_outtake = 0;
            // sus_outtake = true;
            break;
        case Body::PREP_SCORE:
            c_intake = 127;
            c_storage = -127;
            c_outtake = 60;
            // sus_outtake = 1;
            break;
        case Body::DOWN_TOP:
            c_intake = 127;
            c_storage = -127;
            c_outtake = -127;
        case Body::NOTHING:
            c_intake = 0;
            c_outtake = 0;
            c_storage = 0;
            break;
    }

    set_body(c_intake, c_storage, c_outtake, sus_outtake);

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
    while (comp_state != CompState::DISABLE) {
        if (comp_state == CompState::REST) set_body(0, 0, 0, 0);
        else update_body();
		delay_for(pollrate);
    }
}
