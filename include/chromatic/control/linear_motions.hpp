#pragma once
#include "chromatic/control/slew.hpp"
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

        SlewRate fwd_slew;
        SlewRate turn_slew;

        double last_fwd;
        double last_turn;
    public:


        struct Chain {
            double range;
            double min_speed;
            double next_fwd;
            double next_turn;
        };

        MotionController(
            Differential &drivebase, EncodersIMU &localizer, PID fwd_pid, PID turn_pid, SlewRate fwd_slew = SlewRate(), SlewRate turn_slew = SlewRate()):
            drivebase{drivebase}, localizer{localizer}, fwd_pid{fwd_pid}, turn_pid{turn_pid}, fwd_slew{fwd_slew}, turn_slew{turn_slew}
        {
            in_motion = false;
            last_fwd = 0;
            last_turn = 0;
            set_pollrate(20);
        }

        // reset all controllers
        void full_reset() {
            fwd_pid.reset();
            turn_pid.reset();
            fwd_slew.ready();
            turn_slew.ready();
            last_fwd = 0;
            last_turn = 0;
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
            last_fwd = 0;
            last_turn = 0;
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


        // drives relative to the current heading by some amount command, returns true if successful
        // timeout = -1 will simply disable timeout, same as max_speed
        // mono_move will return the moment the robot has reached the large settle range and crossed the target, does not brake, also ensures that robot motor commands are always in the same direction
        // ensure_facing will cause the robot to turn to face the original direction if successful settle. if false, will turn extra only if time allots it
        bool move_by(double amount, ms timeout = -1, double max_speed = -1, bool mono_move = false, Chain chainer = {-1, 0, 0, 0}) {
            if (in_motion) return amount;
            in_motion = true;

            Pose pre_motion = localizer.get_pose();
            Pose target_pose = Pose::project(pre_motion, amount);

            fwd_pid.set_timeout(timeout);
            turn_pid.set_timeout(timeout);

            fwd_pid.reset();
            turn_pid.reset();

            // support for motion chaining
            fwd_slew.ready(last_fwd);
            turn_slew.ready(last_turn);

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
                return 2*turn_error;
            };

            auto get_absolute_error = [&] {
                return mag(localizer.get_pose().pos - target_pose.pos);
            };

            double prev_fwd_error = 0;
            while (!fwd_pid.done() && in_motion) {
                // mind the signs
                // you might want to set target to 0 so that you feed in negatives values to pid so that the output is positive

                // Error Calculations
                double fwd_error = get_fwd_error();
                double turn_error = get_turn_error();
                double abs_error = get_absolute_error();
                bool disable_turn = fabs(fwd_error) <= drivebase.track_width; // prevent swivels when close to target

                // PID & Slew
                double fwd = fwd_pid.compute(fwd_error);
                double turn = turn_pid.compute(turn_error);

                fwd = fwd_slew.update(fwd);
                turn = turn_slew.update(turn);

                // End Behavior
                if (disable_turn) {
                    turn_pid.reset_integral();
                    turn = 0;
                }

                if (chainer.range > 0 && fabs(fwd_error) <= chainer.range) {
                    if (fabs(fwd) < chainer.min_speed) fwd = sign(fwd) * chainer.min_speed;

                    double chain_amt = 1.0 - fabs(fwd_error) / chainer.range;
                    fwd = lerp(fwd, chainer.next_fwd, chain_amt);
                    turn = lerp(turn, chainer.next_turn, chain_amt);
                }

                // exit earlier
                if (mono_move || chainer.range > 0) {
                    bool fwd_error_flip = signflip(fwd_error, prev_fwd_error);
                    bool in_bounds = fwd_pid.get_loose_sc().get_settling();

                    // crossed the threshold and are within a bounds
                    if (fwd_error_flip && in_bounds) {
                        in_motion = false;
                        if (chainer.range < 0) drivebase.brake(); //only brake if not chaining
                        return get_absolute_error();
                    }
                }

                // Scaling
                if (max_speed > 0 && fabs(fwd) > 0 && fabs(fwd) > max_speed) {
                    double ratio = max_speed / fabs(fwd);
                    fwd *= ratio;
                    turn *= ratio;
                }

                // Actuation
                drivebase.command_velocities(fwd, turn);
                last_turn = turn;
                last_fwd = fwd;

                prev_fwd_error = fwd_error;
                delay_for(pollrate);
            }

            // ms time_remaining = fwd_pid.time_left();
            // if (ensure_facing && fwd_pid.settled() && time_remaining > 0) {
            //     turn_to(pre_motion.dir, time_remaining, false);
            // }

            if (chainer.range < 0) {
                drivebase.brake();
                last_fwd = 0;
                last_turn = 0;
            }

            in_motion = false;
            return get_absolute_error();
        }

        // turn to some target radian angle with either specified direction or closest (default), custom timeout (-1 for no timeout) and mono-movement for motion-chaining
        // mono movement will cause turn to exit if we've crossed the moment we cross the target
        double turn_to(double target_radians, ms timeout = -1, bool mono_move = false, DIR direction = DIR::EITHER, Chain chainer = {-1, 0, 0, 0}) {
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
            turn_slew.ready(last_turn);

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

                // error calculations
                double error = get_error();

                // pid & slew
                double turn = turn_pid.compute(error);
                double fwd = 0;

                turn = turn_slew.update(turn);

                if (chainer.range > 0 && fabs(error) <= chainer.range) {
                    if (fabs(turn) < chainer.min_speed) turn = sign(turn) * chainer.min_speed;

                    double chain_amt = 1.0 - fabs(error) / chainer.range;
                    turn = lerp(turn, chainer.next_turn, chain_amt);
                    fwd = lerp(fwd, chainer.next_fwd, chain_amt);
                }

                // early exit
                if (mono_move || chainer.range > 0) {
                    bool error_flip = signflip(error, prev_error);
                    bool in_bounds = turn_pid.get_loose_sc().get_settling();

                    if (error_flip && in_bounds) {
                        in_motion = false;
                        if (chainer.range < 0) drivebase.brake();
                        return true_error();
                    }
                }

                //actuation
                drivebase.command_velocities(fwd, turn);
                last_turn = turn;
                last_fwd = fwd;

                prev_error = error;
                delay_for(pollrate);
            }

            double final_error = true_error();

            if (chainer.range < 0) {
                drivebase.brake();
                last_turn = 0;
                last_fwd = 0;
            }

            in_motion = false;
            return final_error;
        }

    };



}
