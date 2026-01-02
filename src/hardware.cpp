#include "hardware.h"

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
