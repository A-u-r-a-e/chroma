#pragma once
#include "api.h"
#include "chromatic.hpp"
#include "config.h"

enum struct CompState{REST, AUTON, OPCONTROL};
enum struct Body{NOTHING, I_ONLY, I_STORAGE, E_FULL, S_MIDDLE, S_LOW, S_FULL, M_REFRESH, PREP_SCORE};
enum struct Pneumatic{RETRACTED, EXTENDED};

extern std::atomic<CompState> comp_state;
extern std::atomic<Body> body_state;
extern std::atomic<Pneumatic> loader_state;
extern std::atomic<Pneumatic> hook_state;
extern std::atomic<bool> loader_smashing;
extern std::atomic<ms> loader_unsmash_time;
extern std::atomic<bool> damp_out;

void set_body(int cmd_intake, int cmd_storage, int cmd_outtake, bool hold_outtake = false);
void update_body();
void run_body(ms pollrate = 10);
