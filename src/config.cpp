#include "config.h"

const ms OP_POLL_RATE = 5;
const double MAX_ACC = 20;
const double MAX_ALPHA = PI;

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

double LIDAR_RANGE = 100;
ms auton_pollrate = 10;
ms op_pollrate = 10;

Differential drivebase(left_mg, right_mg, 2.75, 0.4, 13, 0.5);

EncodersIMU odometry(drivebase, inertial, 1);
PID fwd_pid(
    6, 0.01, 0.275, 5,
    SettleCondition(0.1, 50), SettleCondition(0.5, 100),
    10, 25, 50
);
PID turn_pid(
    7.1, 0.3, 0.280, to_rad(60),
    SettleCondition(to_rad(1), 50), SettleCondition(to_rad(2), 100),
    2*PI, 2*PI, 8*PI
);

MotionController chassis(drivebase, odometry, fwd_pid, turn_pid, SlewRate{MAX_ACC}, SlewRate{MAX_ALPHA});
