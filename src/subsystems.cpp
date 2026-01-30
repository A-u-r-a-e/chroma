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

std::atomic<bool> damp_out{false};

void set_body(int cmd_intake, int cmd_storage, int cmd_outtake, bool hold_outtake) {
    double volt_rpm = 200.0 / 127.0;
    outtake.set_brake_mode(hold_outtake ? HOLD : BRAKE);
    if (cmd_intake) intake.move(cmd_intake); else intake.brake();
    if (cmd_storage) storage.move_velocity(cmd_storage * volt_rpm); else storage.brake();
    if (cmd_outtake) outtake.move_velocity(cmd_outtake * volt_rpm); else outtake.brake();
}

void update_body() {
    const int ping = lidar.get_distance();
    const int ballin = static_cast<int>(ping < LIDAR_RANGE);

    static ms last_seen = now();
    static bool override_storage = false;

    // actuation

    int c_intake{0}, c_storage{0}, c_outtake{0}, sus_outtake{0};
    switch (body_state.load()) {
        case Body::I_ONLY:
            c_intake = 127;
            c_storage = 0;
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
            c_outtake = (damp_out ? 20 : 127);
            break;
        case Body::S_MIDDLE:
            c_intake = 127;
            c_storage = 127;
            c_outtake = (damp_out ? -20 : -127);
            break;
        case Body::S_LOW:
            c_intake = (damp_out ? -20 : -127);
            c_storage = -40;
            c_outtake = 0;
            break;
        case Body::M_REFRESH:
            c_intake = -127;
            c_storage = -127;
            c_outtake = 127;
            break;
        case Body::I_STORAGE:
            c_intake = 127;
            c_storage = 0;
            c_outtake = 0;
            break;
        case Body::PREP_SCORE:
            c_intake = 127;
            c_storage = STORAGE_SPEED * (3.0/3.0);
            c_outtake = 0;
            sus_outtake = 1;
            break;
        case Body::NOTHING:
            c_intake = 0;
            c_outtake = 0;
            c_storage = 0;
            break;
    }

    // State Implementation
    // if we don't see a block, we don't do storage
    // if its been over a timeout since we last saw a block during storing,  we don't do storage
    if (!ballin || now() > last_seen + STORAGE_TIMEOUT) override_storage = false;
    // if we see a block during storing, we do storage and store the time
    if (ballin && body_state == Body::I_STORAGE) {
        override_storage = true;
        last_seen = now();
    }

    // do the override
    if (override_storage) {
        c_storage = STORAGE_SPEED;
        c_outtake = 2;
        sus_outtake = false;
    }

    set_body(c_intake, c_storage, c_outtake, sus_outtake);

    // loader smashing logic
    if (loader_smashing) {
        if (loader_unsmash_time < now()) {
            loader_smashing = false;
            loader_state = Pneumatic::RETRACTED;
        } else {
            loader_state = Pneumatic::EXTENDED;
        }
    }

    master.print(0, 0, "str: %d, %d", static_cast<int>(override_storage), static_cast<int>(body_state != Body::I_STORAGE && now() > last_seen));


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
