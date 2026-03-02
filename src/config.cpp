#include "config.hpp"

// Robot is 14.125 by 15.125 inches!

enum autons auton_select{LEFT};
std::atomic<CompState> comp_state{CompState::REST};

const ms OP_POLL_RATE = 5;
const ms STORAGE_TIMEOUT = 200;
const ms auton_pollrate = 10;
const ms op_pollrate = 10;
const double MAX_ACC = 20;
const double MAX_ALPHA = PI;
const int STORAGE_SPEED = 80;
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
pros::Distance watcher(8);
pros::Optical light(10);

using namespace chromatic;
const ms TOF::UPDATE_PERIOD = 35;

const MotionController::Chain TURN_TO_FWD = {to_rad(15), to_rad(50), 50, 0};
const MotionController::Chain TURN_TO_BACK = {to_rad(15), to_rad(50), -50, 0};
const MotionController::Chain DRIVE_TO_CW =  {5, 20, 0, to_rad(-60)};
const MotionController::Chain DRIVE_TO_CCW =  {5, 20, 0, to_rad(60)};
const MotionController::Chain SUSF = {15, 40, 40, 0};
const MotionController::Chain SUSB = {15, 40, -40, 0};
const MotionController::Chain FWD_SLOWDOWN = {15, 40, 10, 0};
const MotionController::Chain BACK_SLOWDOWN = {15, 40, -10, 0};
const MotionController::Chain BFWD = {5, 40, -50, 0};
const MotionController::Chain FBACK = {5, 40, -50, 0};

// fake ports and sensors
// sides are both 3.8 offset
// right is 0.5 more than left
// right to left is 10.5
TOF tof_left(3, 5.8, 10);
TOF tof_lfront(4, -2.3, 10);
TOF tof_right(1, 5.85, 10);
TOF tof_rfront(2, -3.2, 10);

Differential drivebase(left_mg, right_mg, 3.25, (24.0/23.5)*(48.0/72.0), (12), 0.5);

std::unique_ptr<Odometry> localizer(new EncodersIMU(drivebase, inertial));

FieldWalls walls{71, 71, 71, 71};
std::vector<FieldElementBox> elements =  {
    FieldElementBox{-25, 25, 43, 51}, // 90º long goal
    FieldElementBox{-25, 25, -51, -43}, // 270º long goal
    FieldElementBox{-10, 10, -10, 10}, // center goals
    FieldElementBox{walls.right - 5, walls.right, 44.5, 49.5}, // 45º loader
    FieldElementBox{walls.left, walls.left + 5, 44.5, 49.5}, // 135º loader
    FieldElementBox{walls.left, walls.left + 5, -49.5, -44.5}, // 225º loader
    FieldElementBox{walls.right - 5, walls.right, -49.5, -44.5}, // 315º loader
};


DoubleTOF front_lidar(tof_lfront, tof_rfront, 5.0, 5.5);
SingleTOF left_lidar(tof_left, 3.8);
SingleTOF right_lidar(tof_right, 3.8);


PID fwd_pid(
    7.3, 0.2, 0.63, 10,
    SettleCondition(0.15, 50),
    SettleCondition(0.25, 150),
    10, 50, 50
);

PID turn_pid(
    10.8, 2.96, 0.69, to_rad(30),
    SettleCondition(to_rad(1), 50),
    SettleCondition(to_rad(2), 100),
    2*PI, 2*PI, 2.5*PI
);

PID head_pid(
    10.8, 2.96, 0.69, to_rad(30),
    SettleCondition(to_rad(1), 50),
    SettleCondition(to_rad(2), 100),
    2*PI, 2*PI, 2.5*PI
);

PID swing_pid(
    82, 8, 4, to_rad(45),
    SettleCondition(to_rad(1), 50),
    SettleCondition(to_rad(2), 100),
    30, 120, 120
);

SlewRate fwd_slew(MAX_ACC);
SlewRate turn_slew(MAX_ALPHA);
SlewRate head_slew(MAX_ALPHA);
SlewRate swing_slew(MAX_ACC*2);

MotionController chassis(drivebase, localizer,
    fwd_pid, turn_pid, head_pid, swing_pid,
    fwd_slew, turn_slew, head_slew, swing_slew
);
