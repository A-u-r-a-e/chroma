#pragma once
#include "api.h"
#include "chromatic.hpp"

// we keep the function with the enum because it helps with debugging
extern enum autons {LEFT=0, RIGHT=1, RIGHT_RUSH=2, SKILLS=3, SOLO=4, CIRCLE=5, TUNE=6} auton_select;
inline const char* get_auton_name(autons auton) {
    switch (auton) {
    case LEFT: return "L 1+6";
    case RIGHT: return "R 0.5+6.5";
    case RIGHT_RUSH: return "RRush 0+4 ";
    case SKILLS: return "Skills";
    case CIRCLE: return "60in Square";
    case TUNE: return "Tuning";
    default: return "Unknown";
    }
}
extern const ms OP_POLL_RATE;
extern const ms STORAGE_TIMEOUT;
extern const double MAX_ACC;
extern const double MAX_ALPHA;
extern const int STORAGE_SPEED;
extern const double LIDAR_RANGE;
extern const double DRIVE_CURVE;

extern const ms auton_pollrate;
extern const ms op_pollrate;

extern pros::Controller master;
extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;

extern pros::Motor intake;
extern pros::Motor storage;
extern pros::Motor outtake;
extern pros::adi::Pneumatics hook;
extern pros::adi::Pneumatics loader;

extern pros::IMU inertial;
extern pros::Distance lidar;
extern pros::Optical light;

extern chromatic::Differential drivebase;
extern chromatic::EncodersIMU odometry;
extern chromatic::PID fwd_pid;
extern chromatic::PID turn_pid;
extern chromatic::MotionController chassis;
