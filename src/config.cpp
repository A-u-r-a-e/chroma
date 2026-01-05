#include "config.h"

const ms OP_POLL_RATE = 5;

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-11, -6, 7}, pros::MotorGearset::blue, pros::MotorEncoderUnits::counts);
pros::MotorGroup right_mg({18, 16, -17}, pros::MotorGearset::blue, pros::MotorEncoderUnits::counts);

pros::Motor intake(14);
pros::Motor storage(15);
pros::Motor outtake(-20);
pros::adi::Pneumatics ear('h', false);
pros::adi::Pneumatics loader('g', false);

pros::IMU inertial(19);
pros::Distance lidar(8);
pros::Optical light(10);

using namespace chromatic;

double LIDAR_RANGE = 100;
ms auton_pollrate = 10;
ms op_pollrate = 10;

Differential drivebase(left_mg, right_mg, 2.75, 1.5, 13, 1.0);

EncodersIMU odometry(drivebase, inertial);

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

MotionController auton_chassis(drivebase, odometry, fwd_pid, turn_pid);
