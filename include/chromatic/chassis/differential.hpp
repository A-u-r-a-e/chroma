#pragma once
#include "api.h"
#include "chromatic/core.hpp"
#include "chromatic/shorthands.hpp"

namespace chromatic {

    struct Differential {
    protected:
        double ticks_per_wheel_rev;
        double max_motor_rpm;
        double inch_mvolts;
        double max_speed;
        double motor_ticks = 0;

        inline double max_speed_ratio(double left, double right) {
            double max_scalar = std::max(fabs(left), fabs(right));
            if (max_scalar == 0) return 1.0;
            return (safety_limiter * max_speed) / max_scalar;
        }
    public:

        pros::MotorGroup &left_mg;
        pros::MotorGroup &right_mg;
        const double wheel_radius;
        const double motor_wheel_ratio;
        const double track_width;
        const double safety_limiter;


        /**
         * @brief Construct a new Differential object
         *
         * @param left_mg reference to left motor groups. all drivebase motors must have the same cartridge
         * @param right_mg reference to left motor groups. all drivebase motors must have the same cartridge
         * @param wheel_radius wheel radius in inches
         * @param gear_ratio motor gear divided by wheel gear
         * @param track_width distance between the centers of the two wheel groups
         * @param safety_limiter proportion of max speed to use (good for stability purposes). less than or equal to 1.0
         */
        Differential(
            pros::MotorGroup &left_mg, pros::MotorGroup &right_mg, double wheel_radius, double gear_ratio, double track_width, double safety_limiter = 1.0):
            left_mg(left_mg), right_mg(right_mg), wheel_radius(wheel_radius), motor_wheel_ratio(gear_ratio), track_width(track_width),  safety_limiter(safety_limiter)
        {
            switch (left_mg.get_gearing()) {
                case pros::MotorGears::red: max_motor_rpm = 100; motor_ticks = 1800; break;
                case pros::MotorGears::green: max_motor_rpm = 200; motor_ticks = 900; break;
                case pros::MotorGears::blue: max_motor_rpm = 600; motor_ticks = 300; break;
            }

            ticks_per_wheel_rev = motor_ticks / motor_wheel_ratio;

            max_speed = max_motor_rpm * motor_wheel_ratio * (PI * wheel_radius) * (1.0 / 60);
            pros::lcd::print(4, "max speed %f", max_speed);

            inch_mvolts = 12000.0 / max_speed;
        }

        // simple getter
        inline double get_ticks_per_wheel_rev() const {
            return ticks_per_wheel_rev;
        }

        // reset the dt motors through tare
        void reset() {
            left_mg.tare_position_all();
            right_mg.tare_position_all();
        }

        // stop the robot with coast, but can brake if parameter is provided
        void brake(bool force = false) {
            auto lbrake = left_mg.get_brake_mode_all();
            auto rbrake = right_mg.get_brake_mode_all();
            // left_mg.set_brake_mode_all(force ? HOLD : COAST);
            // right_mg.set_brake_mode_all(force ? HOLD: COAST);
            left_mg.set_brake_mode_all(HOLD);
            right_mg.set_brake_mode_all(HOLD);
            left_mg.brake();
            right_mg.brake();
            for (int lmb = 0; lmb < lbrake.size(); lmb++) left_mg.set_brake_mode(lbrake[lmb], lmb);
            for (int rmb = 0; rmb < rbrake.size(); rmb++) right_mg.set_brake_mode(rbrake[rmb], rmb);
        }

        void reset_brake() {
            left_mg.set_brake_mode_all(COAST);
            right_mg.set_brake_mode_all(COAST);
        }

        // this prioritizes fwd over turn [-127, 127], turn is right (cw)
        // good for opcontrol
        void arcade_drive(int fwd, int turn) {
            left_mg.move(fwd + turn);
            right_mg.move(fwd - turn);
        }

        // deals in inches/second, this prioritizes turn over fwd, turn is left (ccw)
        // heuristic control, good for non-motion-profile auton
        // WARNING: TURN IS IN THE SAME SCALES AS FWD
        void command_heuristic(double fwd, double turn) {
            double left_cmd = fwd - turn, right_cmd = fwd + turn;
            double ratio = max_speed_ratio(left_cmd, right_cmd);

            if (ratio < 1.0) {
                left_cmd *= ratio;
                right_cmd *= ratio;
            }

            pros::lcd::print(6, "lvolt, %f", inch_mvolts * left_cmd);
            pros::lcd::print(7, "rvolts, %f", inch_mvolts * right_cmd);

            left_mg.move_voltage(inch_mvolts * left_cmd);
            right_mg.move_voltage(inch_mvolts * right_cmd);
        }

        // deals in inches/second, this does inverse kinematics for linear and angular velocity, turn is left (ccw)
        // good for autonomous with velocity profiling, as this (tries to) actuate to motion profiled velocities
        void command_velocities(double linear, double angular, bool respect_max_speed = false) {
            /*
                We know that Arc Length s = theta * radius
                Therefore, Radius, R = s / theta

                We can find the equivalent for angular and linear velocity
                Let s(t) and theta(t),

                ds/dt = dtheta/dt * radius
                v = w * radius

                Therefore, R = v / w.

                If we are turning to the left, the left side is closer to the center of the turning circle
                Therefore, R_left = R - D / 2, where D is the cross track width, and R_right = R + D / 2

                Since we are given both v and w (linear, angular),
                We can find the differential velocities for left and right motor groups, using v = w * radius

                Intuitively, we can imagine this in the arc length form,
                Understand it as finding the arc length of each motor group when they travel alongst this circle

                Therefore, v_left = w * (R - D / 2) and v_right = w * (R + D / 2).

                As R = linear/angular, or v / w, we can expand and simplify these expressions to:
                v_left = v - w * D / 2, v_right = v + w * D / 2
            */

            double v_left = linear - angular * track_width / 2;
            double v_right = linear + angular * track_width / 2;

            command_left_only(v_left, respect_max_speed);
            command_right_only(v_right, respect_max_speed);
        }

        // command each side of the drivetrain discreetly.
        void command_left_only(double v_left, bool respect_max_speed) {
            if (respect_max_speed && fabs(v_left) > max_speed) {
                v_left = sign(v_left) * max_speed;
            }

            left_mg.move_voltage(inch_mvolts * v_left);
        }

        void command_right_only(double v_right, bool respect_max_speed) {
            if (respect_max_speed && fabs(v_right) > max_speed) {
                v_right = sign(v_right) * max_speed;
            }

            right_mg.move_voltage(inch_mvolts * v_right);
        }

        void command_brake(DIR sides = DIR::EITHER) {
            left_mg.set_brake_mode_all(HOLD);
            right_mg.set_brake_mode_all(HOLD);
            if (sides == DIR::LEFT || sides == DIR::EITHER) {
                left_mg.brake();
            }
            if (sides == DIR::RIGHT || sides == DIR::EITHER) {
                right_mg.brake();
            }
        }
    };
}
