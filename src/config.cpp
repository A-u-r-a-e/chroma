#include "config.h"

using namespace chromatic;

Differential drivetrain(left_mg, right_mg, 2.75, 1.5, 13, 1.0);

EncodersIMU odometry(drivetrain, inertial, 440);

PID fwd_pid(
    3.7, 0.05, 1.7, 3,
    SettleCondition(1.0, 100), SettleCondition(2.0, 500),
    20, 30, 60
);

PID turn_pid(
    0.95, 0.03, 0.6, to_rad(30),
    SettleCondition(to_rad(2), 100), SettleCondition(to_rad(5), 500),
    to_rad(120), to_rad(240), to_rad(480)
);

MotionController auton_controller(drivetrain, odometry, fwd_pid, turn_pid);
