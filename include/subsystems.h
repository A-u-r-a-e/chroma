#pragma once
#include "api.h"
#include "chromatic.hpp"
#include "config.h"

enum struct CompState{REST, AUTON, OPCONTROL};
enum struct Body{NOTHING, I_ONLY, I_STORAGE, E_FULL, S_MIDDLE, S_LOW, S_FULL, M_REFRESH};
enum struct Pneumatic{RETRACTED, EXTENDED};

extern std::atomic<CompState> comp_state;
extern std::atomic<Body> body_state;
extern std::atomic<Pneumatic> loader_state;
extern std::atomic<Pneumatic> hook_state;

void set_body(int cmd_intake, int cmd_storage, int cmd_outtake);
void update_body();
void run_body(ms pollrate = 10);
