#include "config.h"
#include "pros/abstract_motor.hpp"

const ms OP_POLL_RATE = 5;
const ms STORAGE_TIMEOUT = 200;
const ms auton_pollrate = 10;
const ms op_pollrate = 10;
const double MAX_ACC = 20;
const double MAX_ALPHA = PI;
const int STORAGE_SPEED = 60;
const double LIDAR_RANGE = 70;
const double DRIVE_CURVE = 1.5;

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-11, -6, 7}, pros::MotorGearset::blue, pros::MotorEncoderUnits::counts);
pros::MotorGroup right_mg({18, 16, -17}, pros::MotorGearset::blue, pros::MotorEncoderUnits::counts);

pros::Motor intake(14);
pros::Motor storage(15);
pros::Motor outtake(-20);
pros::adi::Pneumatics hook('h', true);
pros::adi::Pneumatics loader('g', false);

pros::IMU inertial(19);
pros::Distance lidar(8);
pros::Optical light(10);

using namespace chromatic;

Differential drivebase(left_mg, right_mg, 3.25, (24.0/23.5)*(48.0/72.0), (12), 0.5);

EncodersIMU odometry(drivebase, inertial);
PID fwd_pid(
    7.3,0.2, 0.63, 10,
    SettleCondition(0.1, 50),
    SettleCondition(0.7, 150),
    10, 50, 50
);

PID turn_pid(
    10.8, 2.96, 0.69, to_rad(30),
    SettleCondition(to_rad(1), 50),
    SettleCondition(to_rad(3), 100),
    2*PI, 2*PI, 2.5*PI
);

MotionController chassis(drivebase, odometry, fwd_pid, turn_pid, SlewRate{MAX_ACC}, SlewRate{MAX_ALPHA});
