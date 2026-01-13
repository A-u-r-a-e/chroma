#pragma once
#include "chromatic/control/slew.hpp"
#include "chromatic/core/helpers.hpp"
#include "chromatic/shorthands.hpp"
#include "chromatic/core.hpp"
#include "chromatic/chassis.hpp"
#include "chromatic/control/pid.hpp"
#include <cmath>

namespace chromatic {

    struct MotionController {
    private:
        Differential &drivebase;
        EncodersIMU &localizer;
        PID fwd_pid, turn_pid;

        std::atomic<bool> in_motion;
        ms pollrate;

        SlewRate fwd_slew;
        SlewRate turn_slew;
    public:

        MotionController(
            Differential &drivebase, EncodersIMU &localizer, PID fwd_pid, PID turn_pid, SlewRate fwd_slew = SlewRate(), SlewRate turn_slew = SlewRate()):
            drivebase{drivebase}, localizer{localizer}, fwd_pid{fwd_pid}, turn_pid{turn_pid}, fwd_slew{fwd_slew}, turn_slew{turn_slew}
        {
            in_motion = false;
            set_pollrate(20);
        }

        // reset all controllers
        void full_reset() {
            fwd_pid.reset();
            turn_pid.reset();
            fwd_slew.ready();
            turn_slew.ready();
            in_motion = false;
        }

        // set pollrate/tickrate
        void set_pollrate(ms pollrate) {
            this->pollrate = pollrate;
        }

        // set max fwd slew acceleration
        void set_fwd_slew(double max_acc) {
            fwd_slew.set_limit(max_acc);
        }

        // set max turn slew ang. acceleration
        void set_turn_slew(double max_alpha) {
            turn_slew.set_limit(max_alpha);
        }

        // if MotionController is active or not
        inline bool is_in_motion() {
            return in_motion;
        }

        // interupt motion
        inline void interrupt() {
            in_motion = false;
            drivebase.brake();
        }

        // command outside motion, this is motor output based
        inline void override_arcade(double fwd, double turn) {
            drivebase.arcade_drive(fwd, turn);
        }

        // command outside motion, this scales fwd to irl speeds
        inline void override_heuristic(double fwd, double turn) {
            drivebase.command_heuristic(fwd, turn);
        }

        // command outside motion, this does reverse kinematics
        inline void override_velocities(double linear, double angular, bool respect_max_speed = false) {
            drivebase.command_velocities(linear, angular, respect_max_speed);
        }

        // brake outside motion
        inline void override_brake() {
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


        // move towards a position, could be pose as well. if just position, the dir you give target doesn't matter
        // direction has positive meaning forwards yeah
        // timeout is timeout -1 means none
        // mono move just means do we allow the reverse movement. good for motion "chaining"
        // also pose determines if we care about end heading
        [[deprecated("this is buggy and does not work")]]
        double move_to(Pose target, SIGN direction, ms timeout = -1, bool mono_move = false, bool also_pose = false) {
            auto true_error = [&] {
                return mag(target.pos - localizer.get_pose().pos);
            };

            if (in_motion || direction == SIGN::ZERO) return true_error();
            in_motion = true;

            Pose pre_motion = localizer.get_pose();
            Pose target_pose = target;
            bool backwards = direction == SIGN::NEGATIVE;

            fwd_pid.set_timeout(timeout);
            turn_pid.set_timeout(timeout);

            fwd_pid.reset();
            turn_pid.reset();

            fwd_slew.ready();
            turn_slew.ready();

            auto get_fwd_error = [&] {
                Vec displacement = target_pose.pos - localizer.get_pose().pos;
                double component_on_axis = dot(displacement, Vec::Polar(localizer.get_pose().dir));
                return component_on_axis;
            };

            auto get_turn_error = [&] {
                Pose cur_pose = localizer.get_pose();
                double target_facing = (target_pose.pos - cur_pose.pos).angle();
                // if we're moving backwards we want to face away
                if (backwards) target_facing = wrap_angle(target_facing + PI);
                double turn_error = calculate_turn(cur_pose.dir, target_facing);
                return turn_error;
            };

            double prev_fwd_error = 0;
            while (!fwd_pid.done() && in_motion) {
                // mind the signs
                // you might want to set target to 0 so that you feed in negatives values to pid so that the output is positive

                double fwd_error = get_fwd_error();
                double turn_error = get_turn_error();
                bool disable_turn = fabs(fwd_error) <= drivebase.track_width; // prevent swivels when close to target

                double fwd = fwd_pid.compute(fwd_error);
                double turn = turn_pid.compute(turn_error);

                fwd = fwd_slew.update(fwd);
                turn = turn_slew.update(turn);

                if (disable_turn) {
                    turn_pid.reset_integral();
                    turn = 0;
                }

                // if we are ensuring that a) we exit after crossing threshold and b) we do not move in opposite direction
                if (mono_move) {
                    bool fwd_error_flip = signflip(fwd_error, prev_fwd_error);
                    bool in_bounds = fwd_pid.get_loose_sc().get_settling();

                    // crossed the threshold and are within a bounds
                    if (fwd_error_flip && in_bounds) {
                        in_motion = false;
                        drivebase.brake();
                        return true_error();
                    }
                }

                drivebase.command_velocities(fwd, turn);

                prev_fwd_error = fwd_error;
                delay_for(pollrate);
            }

            ms time_remaining = fwd_pid.time_left();
            if (also_pose && fwd_pid.settled() && time_remaining > 0) {
                turn_to(target_pose.dir, time_remaining, false);
            }


            drivebase.brake();
            in_motion = false;
            return true_error();

        }

        // drives relative to the current heading by some amount command, returns true if successful
        // timeout = -1 will simply disable timeout, same as max_speed
        // mono_move will return the moment the robot has reached the large settle range and crossed the target, does not brake, also ensures that robot motor commands are always in the same direction
        // ensure_facing will cause the robot to turn to face the original direction if successful settle. if false, will turn extra only if time allots it
        bool move_by(double amount, ms timeout = -1, double max_speed = -1, bool mono_move = false, bool ensure_facing = false) {
            if (in_motion) return amount;
            in_motion = true;

            Pose pre_motion = localizer.get_pose();
            Pose target_pose = Pose::project(pre_motion, amount);

            fwd_pid.set_timeout(timeout);
            turn_pid.set_timeout(timeout);

            fwd_pid.reset();
            turn_pid.reset();

            fwd_slew.ready();
            turn_slew.ready();

            auto get_fwd_error = [&] {
                Vec displacement = target_pose.pos - localizer.get_pose().pos;
                double component_on_axis = dot(displacement, Vec::Polar(localizer.get_pose().dir));
                return component_on_axis;
            };

            auto get_turn_error = [&] {
                Pose cur_pose = localizer.get_pose();
                double target_facing = (target_pose.pos - cur_pose.pos).angle();
                // if we're moving backwards we want to face away
                if (amount < 0) target_facing = wrap_angle(target_facing + PI);
                double turn_error = calculate_turn(cur_pose.dir, target_facing);
                return turn_error;
            };

            auto get_absolute_error = [&] {
                return mag(localizer.get_pose().pos - target_pose.pos);
            };

            double prev_fwd_error = 0;
            while (!fwd_pid.done() && in_motion) {
                // mind the signs
                // you might want to set target to 0 so that you feed in negatives values to pid so that the output is positive

                double fwd_error = get_fwd_error();
                double turn_error = get_turn_error();
                double abs_error = get_absolute_error();
                bool disable_turn = fabs(abs_error) <= drivebase.track_width; // prevent swivels when close to target

                double fwd = fwd_pid.compute(fwd_error);
                double turn = turn_pid.compute(turn_error);

                fwd = fwd_slew.update(fwd);
                turn = turn_slew.update(turn);

                if (disable_turn) {
                    turn_pid.reset_integral();
                    turn = 0;
                }

                // if we are ensuring that a) we exit after crossing threshold and b) we do not move in opposite direction
                if (mono_move) {
                    bool fwd_error_flip = signflip(fwd_error, prev_fwd_error);
                    bool in_bounds = fwd_pid.get_loose_sc().get_settling();

                    // crossed the threshold and are within a bounds
                    if (fwd_error_flip && in_bounds) {
                        in_motion = false;
                        drivebase.brake();
                        return get_absolute_error();
                    }
                }

                if (max_speed > 0 && fabs(fwd) > 0 && fabs(fwd) > max_speed) {
                    double ratio = max_speed / fabs(fwd);
                    fwd *= ratio;
                    turn *= ratio;
                }

                drivebase.command_velocities(fwd, turn);

                prev_fwd_error = fwd_error;
                delay_for(pollrate);
            }

            ms time_remaining = fwd_pid.time_left();
            if (ensure_facing && fwd_pid.settled() && time_remaining > 0) {
                turn_to(pre_motion.dir, time_remaining, false);
            }

            drivebase.brake();
            in_motion = false;
            return get_absolute_error();
        }

        // turn to some target radian angle with either specified direction or closest (default), custom timeout (-1 for no timeout) and mono-movement for motion-chaining
        // mono movement will cause turn to exit if we've crossed the moment we cross the target
        double turn_to(double target_radians, ms timeout = -1, bool mono_move = false, DIR direction = DIR::EITHER) {
            auto true_error = [&] {
                return calculate_turn(target_radians, localizer.get_pose().dir);
            };
            if (in_motion) return true_error();
            in_motion = true;

            double origin = localizer.get_pose().dir;
            double target = target_radians;
            double amount = calculate_turn(origin, target, direction);
            bool ignore_direction = false;

            turn_pid.set_timeout(timeout);

            turn_pid.reset();
            turn_slew.ready();

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

                double turn = turn_pid.compute(error);

                turn = turn_slew.update(turn);

                if (mono_move) {
                    bool error_flip = signflip(error, prev_error);
                    bool in_bounds = turn_pid.get_loose_sc().get_settling();

                    if (error_flip && in_bounds) {
                        in_motion = false;
                        drivebase.brake();
                        return true_error();
                    }
                }

                drivebase.command_velocities(0, turn);

                prev_error = error;
                delay_for(pollrate);
            }

            drivebase.brake();
            in_motion = false;
            return true_error();
        }

        // turn by some amount with a direction (cannot be EITHER, will exit), custom timeout (-1 for no timeout), and mono-movement for motion-chaining
        // mono movement will cause turn to exit if we've crossed the moment we cross the target
        bool turn_by(double amount_radians, DIR direction, ms timeout = -1, bool mono_move = false) {
            if (in_motion || direction == DIR::EITHER) return amount_radians;
            in_motion = true;

            double amount = amount_radians;

            turn_pid.set_timeout(timeout);

            turn_pid.reset();

            turn_slew.ready();

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

                turn_cmd = turn_slew.update(turn_cmd);

                if (mono_move) {
                    bool error_flip = signflip(error, prev_error);
                    bool in_bounds = turn_pid.get_loose_sc().get_settling();

                    if (error_flip && in_bounds) {
                        in_motion = false;
                        drivebase.brake();
                        return get_error();
                    }
                }

                drivebase.command_velocities(0, turn_cmd, false);

                prev_error = error;
                delay_for(pollrate);
            }

            drivebase.brake();
            in_motion = false;
            return get_error();
        }

    };



}
