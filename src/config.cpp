#include "config.h"

using namespace chromatic;

double LIDAR_RANGE = 100;
ms auton_pollrate = 10;
ms op_pollrate = 10;

Differential drivetrain(left_mg, right_mg, 2.75, 1.5, 13, 1.0);

EncodersIMU odometry(drivetrain, inertial, 440);

PID fwd_pid(
    7, 0.1, 5, 5,
    SettleCondition(0.25, 50), SettleCondition(0.5, 500),
    20, 30, 60
);

PID turn_pid(
    0.88, 0.05, 0.63, to_rad(10),
    SettleCondition(to_rad(1), 50), SettleCondition(to_rad(2), 200),
    to_rad(120), to_rad(180), to_rad(360)
);

MotionController auton_controller(drivetrain, odometry, fwd_pid, turn_pid);
