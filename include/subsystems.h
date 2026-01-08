#pragma once
#include "api.h"
#include "chromatic.hpp"
#include "config.h"

enum struct CompState{REST, AUTON, OPCONTROL};
enum struct Body{NOTHING, I_ONLY, I_STORAGE, E_FULL, S_MIDDLE, S_LOW, S_FULL};
enum struct Pneumatic{RETRACTED, EXTENDED};

extern std::atomic<CompState> comp_state;
extern std::atomic<Body> body_state;
extern std::atomic<Pneumatic> loader_state;
extern std::atomic<Pneumatic> hook_state;

// commands to give the actuators
extern std::atomic<int> intake_cmd;
extern std::atomic<int> outtake_cmd;
extern std::atomic<int> storage_cmd;
extern std::atomic<int> upper_outake;
extern std::atomic<int> scoring;
extern std::atomic<ms> filter_until;

void stop_robot();
void intake_storage_update(const double LIDAR_RANGE, bool slow = false);
void middle_outtake_update();
void suck_update();
void spit_update();
void full_linear_update(chromatic::SIGN direction);

void update_body();
void prepare_body();
void actuate_body();
void run_body(ms pollrate = 10);
void stop_body();
