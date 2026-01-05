#pragma once
#include "chromatic/core/helpers.hpp"
#include "chromatic/shorthands.hpp"
#include "chromatic/core.hpp"
#include "chromatic/chassis.hpp"
#include "chromatic/control/pid.hpp"

namespace chromatic {

    struct MotionController {
    private:
        Differential &drivebase;
        EncodersIMU &localizer;
        PID fwd_pid, turn_pid;

        std::atomic<bool> in_motion;
        ms pollrate;
    public:

        MotionController(
            Differential &drivebase, EncodersIMU &localizer, PID fwd_pid, PID turn_pid):
            drivebase{drivebase}, localizer{localizer}, fwd_pid{fwd_pid}, turn_pid{turn_pid}
        {
            in_motion = false;
            set_pollrate(20);
        }

        // set pollrate/tickrate
        void set_pollrate(ms pollrate) {
            this->pollrate = pollrate;
        }

        // if MotionController is active or not
        bool is_in_motion() {
            return in_motion;
        }

        void interrupt() {
            in_motion = false;
            drivebase.brake();
        }

        /* Motion Code Structure Outline
         *
         * Claim in_motion, or return failure if has already been claimed
         *
         * Define motion key points
         *
         * Set controller timeouts and configs
         *
         * Reset controllers
         *
         * Define lambdas
         *
         * CONTROL LOOP
         *
         * END BEHAVIOR - note that this should not extend timeout
         *
         * Release claim on in_motion
         */

        /* Control Loop Structure Outline
         *
         * Outside loop, define any persistent/cross-iteration variables
         *
         * Calculate errors
         *
         * Compute control commands
         *
         * Check special cases
         *
         * Actuation
         *
         * Update persistent variables
         *
         * Tick time
         */

        // drives relative to the current heading by some amount command, returns true if successful
        // timeout = -1 will simply disable timeout
        // mono_move will return the moment the robot has reached the large settle range and crossed the target, does not brake, also ensures that robot motor commands are always in the same direction
        // ensure_facing will cause the robot to turn to face the original direction if successful settle. if false, will turn extra only if time allots it
        bool move_by(double amount, ms timeout = -1, bool mono_move = false, bool ensure_facing = false) {
            if (in_motion) return false;
            in_motion = true;

            Pose pre_motion = localizer.get_pose();
            Pose target_pose = Pose::project(pre_motion, amount);

            fwd_pid.set_timeout(timeout);
            turn_pid.set_timeout(timeout);

            fwd_pid.reset();
            turn_pid.reset();

            auto get_fwd_error = [&] {
                Vec displacement = target_pose.pos - localizer.get_pose().pos;
                double component_on_axis = dot(displacement, Vec::Polar(localizer.get_pose().dir));
                return component_on_axis;
            };

            auto get_turn_error = [&] {
                Pose cur_pose = localizer.get_pose();
                double target_facing = (target_pose.pos - cur_pose.pos).angle();
                double turn_error = calculate_turn(cur_pose.dir, target_facing);
                return turn_error;
            };

            auto get_facing_error = [&] {
                Pose cur_pose = localizer.get_pose();
                double angle_offset = calculate_turn(
                    cur_pose.dir,
                    target_pose.dir
                );
                return angle_offset;
            };

            double prev_fwd_error = 0;
            while (!fwd_pid.done() && in_motion) {
                // mind the signs
                // you might want to set target to 0 so that you feed in negatives values to pid so that the output is positive

                double fwd_error = get_fwd_error();
                double turn_error = get_turn_error();

                double fwd = fwd_pid.compute(fwd_error);
                double turn = turn_pid.compute(turn_error);

                // if we are ensuring that a) we exit after crossing threshold and b) we do not move in opposite direction
                if (mono_move) {
                    bool fwd_error_flip = fwd_error * prev_fwd_error < 0;
                    bool in_bounds = fwd_pid.get_loose_sc().get_settling();

                    // crossed the threshold and are within a bounds
                    if (fwd_error_flip && in_bounds) {
                        in_motion = false;
                        drivebase.brake();
                        return true;
                    }
                }

                // prevent swivels
                if (fwd_pid.get_tight_sc().get_settling()) turn = 0;

                drivebase.command_heuristic(fwd, turn);

                prev_fwd_error = fwd_error;
                delay_for(pollrate);
            }

            if (fwd_pid.settled()) {
                if (ensure_facing) turn_pid.reset();

                while (!turn_pid.done() && in_motion) {
                    double error = get_facing_error();
                    double turn = turn_pid.compute(error);
                    drivebase.command_heuristic(0, turn);
                    delay_for(pollrate);
                }
            }

            drivebase.brake();
            in_motion = false;
            return true;
        }

        // turn to some target radian angle with either specified direction or closest (default), custom timeout (-1 for no timeout) and mono-movement for motion-chaining
        // // mono movement will cause turn to exit if we've crossed the moment we cross the target
        bool turn_to(double target_radians, ms timeout = -1, bool mono_move = false, DIR direction = DIR::EITHER) {
            if (in_motion) return false;
            in_motion = true;

            double origin = localizer.get_pose().dir;
            double target = target_radians;
            double amount = calculate_turn(origin, target, direction);
            bool ignore_direction = false;

            turn_pid.set_timeout(timeout);

            turn_pid.reset();

            auto get_error = [&] {

                // if we're close enough to ignore the direction
                if (turn_pid.get_loose_sc().get_settling() && !ignore_direction) ignore_direction = true;

                double error = calculate_turn(
                    localizer.get_pose().dir,
                    target,
                    (ignore_direction ? DIR::EITHER : direction)
                );

                return error;
            };

            double prev_error = 0;
            while (!turn_pid.done() && in_motion) {

                double error = get_error();

                double turn_cmd = turn_pid.compute(error);

                if (mono_move) {
                    bool error_flip = error * prev_error < 0;
                    bool in_bounds = turn_pid.get_loose_sc().get_settling();

                    if (error_flip && in_bounds) {
                        in_motion = false;
                        drivebase.brake();
                        return true;
                    }
                }

                drivebase.command_heuristic(0, turn_cmd);

                prev_error = error;
                delay_for(pollrate);
            }

            drivebase.brake();
            in_motion = false;
            return true;
        }

        // turn by some amount with a direction (cannot be EITHER, will exit), custom timeout (-1 for no timeout), and mono-movement for motion-chaining
        // mono movement will cause turn to exit if we've crossed the moment we cross the target
        bool turn_by(double amount_radians, DIR direction, ms timeout = -1, bool mono_move = false) {
            if (in_motion || direction == DIR::EITHER) return false;
            in_motion = true;

            double amount = amount_radians;

            turn_pid.set_timeout(timeout);

            turn_pid.reset();
            double amount_moved = 0;
            double last_dir = localizer.get_pose().dir;

            auto get_error = [&] {
                double dir = localizer.get_pose().dir;

                amount_moved += calculate_turn(last_dir, dir); // assumes high tickrate
                last_dir = dir;

                double error = static_cast<int>(direction) * amount - amount_moved;

                return error;
            };

            double prev_error = 0;
            while (!turn_pid.done() && in_motion) {

                double error = get_error();

                double turn_cmd = turn_pid.compute(error);

                if (mono_move) {
                    bool error_flip = error * prev_error < 0;
                    bool in_bounds = turn_pid.get_loose_sc().get_settling();

                    if (error_flip && in_bounds) {
                        in_motion = false;
                        drivebase.brake();
                        return true;
                    }
                }

                drivebase.command_heuristic(0, turn_cmd);

                prev_error = error;
                delay_for(pollrate);
            }

            drivebase.brake();
            in_motion = false;
            return true;
        }

    };



}
