#pragma once
#include "chromatic/chassis/odometry.hpp"
#include "chromatic/core/helpers.hpp"
#include "chromatic/shorthands.hpp"
#include "chromatic/control/slew.hpp"
#include "chromatic/core.hpp"
#include "chromatic/chassis.hpp"
#include "chromatic/control/pid.hpp"
#include "chromatic/control/cubicspline.hpp"
#include "pros/llemu.hpp"
#include <algorithm>
#include <memory>

namespace chromatic {


    struct MotionController {
    private:
        Differential &drivebase;
        std::unique_ptr<Odometry> &localizer;
        PID fwd_pid, turn_pid, head_pid, swing_pid;

        std::atomic<bool> in_motion;
        ms pollrate;

        SlewRate fwd_slew, turn_slew, head_slew, swing_slew;

        double last_fwd;
        double last_turn;

    public:

        enum struct Exit {MONO, LOOSE, TIGHT};

        struct Chain {
            double range = -1;
            double min_speed = 0;
            double next_fwd = 0;
            double next_turn = 0;

            Chain(double range = -1, double min_speed = 0, double next_fwd = 0, double next_turn = 0): range(range), min_speed(min_speed), next_fwd(next_fwd), next_turn(next_turn)
            {};

            enum Motions{FORWARD, BACKWARD, TURN_CCW, TURN_CW} ;

            Chain(double range, double min_speed, Motions next_motion) {
                this->range = range;
                this->min_speed = min_speed;
                switch (next_motion) {
                case FORWARD: next_fwd = 10; next_turn = 0; break;
                case BACKWARD: next_fwd = -10; next_turn = 0; break;
                case TURN_CCW: next_fwd = 0; next_turn = to_rad(20); break;
                case TURN_CW: next_fwd = 0; next_turn = to_rad(-20); break;
                }
            }

            Chain operator*(const double other) {
                return Chain{range, min_speed, next_fwd * other, next_turn * other};
            }
        };

        struct Prediction {
        private:
            Pose target;
        public:
            Pose get_pose() {return target;}
            Vec get_pos() {return target.pos;}
            double get_heading() {return target.dir;}

            void set_pose(Pose override) {target = override;}
            void set_pos(Vec pos) {target.pos = pos;}
            void set_heading(double heading) {target.dir = heading;}

            void project_fwd(double amount) {target = Pose::project(target, amount);}
            void project_turn(double amount) {target.dir = wrap_angle(target.dir + amount);}
        } cache;

        MotionController(
            Differential &drivebase, std::unique_ptr<Odometry> &localizer, PID fwd_pid, PID turn_pid, PID head_pid, PID swing_pid, SlewRate fwd_slew = SlewRate(), SlewRate turn_slew = SlewRate(), SlewRate head_slew = SlewRate(), SlewRate swing_slew = SlewRate()):
            drivebase{drivebase}, localizer{localizer}, fwd_pid{fwd_pid}, turn_pid{turn_pid}, head_pid{head_pid}, swing_pid{swing_pid}, fwd_slew{fwd_slew}, turn_slew{turn_slew}, head_slew{head_slew}, swing_slew{swing_slew}
        {
            in_motion = false;
            last_fwd = 0;
            last_turn = 0;
            refresh_cache();
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

        // ready the cache for movements
        void refresh_cache() {
            cache.set_pose(localizer->get_pose());
        }

        void set_cache(Pose pose) {
            cache.set_pose(pose);
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
        inline void override_arcade(int fwd, int turn) {
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
        inline void override_brake(bool force = false) {
            drivebase.brake(force);
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

        // neat helper function to move forward for a duration of time in milliseconds
        // specified in terms of inches per second and degrees per second
        bool timed_drive(ms duration, double inch_sec, double deg_sec = 0, bool hard_stop = false) {
            if (in_motion) return false;
            in_motion = true;
            override_velocities(inch_sec, to_rad(deg_sec), false);
            delay_for(duration);
            override_brake(hard_stop);
            refresh_cache();
            in_motion = false;
            return true;
        }

        // drives forward and maintains heading using the turn pid. returns final forwards error
        // setting timeout or max speed to -1 will disable them
        // mono_move ensures motor commands are uni-directional by exiting the function once the robot overshoots and lies within settle range
        // chainer allows for motion chaining. range in inches, min_speed in inches/sec, and some default configuration available as well
        double move_by(double amount, ms timeout = -1, Exit move_type = Exit::LOOSE, double max_speed = -1, Chain chainer = Chain{}) {
            const bool do_chain = chainer.range > 0;

            if (in_motion) return amount;
            in_motion = true;

            cache.project_fwd(amount);
            Pose target_pose = cache.get_pose();

            fwd_pid.set_timeout(timeout);
            head_pid.set_timeout(timeout);

            fwd_pid.reset();
            head_pid.reset();

            // support for motion chaining
            fwd_slew.ready(last_fwd);
            head_slew.ready(last_turn);

            auto get_fwd_error = [&] {
                Vec displacement = target_pose.pos - localizer->get_pose().pos;
                double component_on_axis = dot(displacement, Vec::Polar(localizer->get_pose().dir));
                return component_on_axis;
            };

            auto get_head_error = [&] {
                Pose cur_pose = localizer->get_pose();

                double bearing = (target_pose.pos - cur_pose.pos).angle();
                // if we're moving backwards we want to face away
                if (amount < 0) bearing = wrap_angle(bearing + PI);
                double head_error = calculate_turn(cur_pose.dir, bearing);
                // double cte_correction = calculate_turn(target_pose.dir, bearing); // cross track correction
                double cte_correction = 0;
                return head_error + cte_correction;
            };

            auto get_absolute_error = [&] {
                return mag(localizer->get_pose().pos - target_pose.pos);
            };

            double prev_fwd_error = 0;
            while (!fwd_pid.done(move_type == Exit::TIGHT) && in_motion) {
                // mind the signs
                // you might want to set target to 0 so that you feed in negatives values to pid so that the output is positive

                // Error Calculations
                double fwd_error = get_fwd_error();
                double head_error = get_head_error();
                double abs_error = get_absolute_error();
                bool disable_turn = fabs(abs_error) <= drivebase.track_width / 2; // prevent swivels when close to target

                // PID & Slew
                double fwd = fwd_pid.compute(fwd_error);
                double correction = head_pid.compute(head_error);

                fwd = fwd_slew.update(fwd);
                correction = head_slew.update(correction);

                // End Behavior
                if (disable_turn) {
                    head_pid.reset_integral();
                    correction = 0;
                } else {
                    if (amount > 0) fwd = std::max(fwd, 0.0);
                    else fwd = std::min(fwd, 0.0);
                }

                if (do_chain && fabs(fwd_error) <= chainer.range) {
                    if (fabs(fwd) < chainer.min_speed) fwd = sign(fwd) * chainer.min_speed;

                    double chain_amt = 1.0 - fabs(fwd_error) / chainer.range;
                    fwd = lerp(fwd, chainer.next_fwd, chain_amt);
                    correction = lerp(correction, chainer.next_turn, chain_amt);
                }

                // exit earlier
                if (move_type == Exit::MONO || do_chain) {
                    bool progress_condition = do_chain || signflip(fwd_error, prev_fwd_error);
                    bool in_bounds = fwd_pid.get_loose_sc().get_settling();

                    // crossed the threshold and are within a bounds
                    if (progress_condition && in_bounds) {
                        in_motion = false;
                        if (!do_chain) drivebase.brake(); //only brake if not chaining
                        return get_fwd_error();
                    }
                }

                // Scaling
                if (max_speed > 0 && fabs(fwd) > 0 && fabs(fwd) > max_speed) {
                    double ratio = max_speed / fabs(fwd);
                    fwd *= ratio;
                    correction *= ratio;
                }

                // Actuation
                drivebase.command_velocities(fwd, correction);
                last_turn = correction;
                last_fwd = fwd;

                prev_fwd_error = fwd_error;
                delay_for(pollrate);
            }

            if (!do_chain) {
                drivebase.brake();
                last_fwd = 0;
                last_turn = 0;
            }

            in_motion = false;
            return get_fwd_error();
        }

        // drives towards a point using fwd and turn pid. returns final euclidean error
        // setting timeout or max speed to -1 will disable them
        // mono_move ensures motor commands are uni-directional near settle by exiting the function once the robot overshoots and lies within settle range
        // chainer allows for motion chaining. range in inches, min_speed in inches/sec, and some default configuration available as well
        // this function will cause the cache, which ensures your movement direction, to be the straight line between the current cache point and the target
        double move_to(Pose target_pose, FACE facing = FACE::FWD, ms timeout = -1, double max_speed = -1, Exit move_type = Exit::LOOSE, Chain chainer = Chain{}) {

            Vec target = target_pose.pos;

            const bool do_chain = chainer.range > 0;
            if (in_motion) return mag(target - localizer->get_pose().pos);
            in_motion = true;

            cache.set_pose(target_pose);

            fwd_pid.set_timeout(timeout);
            head_pid.set_timeout(timeout);

            fwd_pid.reset();
            head_pid.reset();

            // support for motion chaining
            fwd_slew.ready(last_fwd);
            head_slew.ready(last_turn);

            auto get_fwd_error = [&] {
                Vec displacement = target_pose.pos - localizer->get_pose().pos;
                double component_on_axis = dot(displacement, Vec::Polar(localizer->get_pose().dir));
                return component_on_axis;
            };

            auto get_head_error = [&] {
                Pose cur_pose = localizer->get_pose();
                double bearing = (target_pose.pos - cur_pose.pos).angle();
                // if we're moving backwards we want to face away
                if (facing == FACE::BACK) bearing = wrap_angle(bearing + PI);
                double head_error = calculate_turn(cur_pose.dir, bearing);
                double cte_correction = calculate_turn(target_pose.dir, bearing); // cross track correction

                return head_error+cte_correction;
            };

            auto get_absolute_error = [&] {
                return mag(localizer->get_pose().pos - target_pose.pos);
            };

            double prev_fwd_error = 0;
            while (!fwd_pid.done(move_type == Exit::MONO) && in_motion) {
                // mind the signs
                // you might want to set target to 0 so that you feed in negatives values to pid so that the output is positive

                // Error Calculations
                double fwd_error = get_fwd_error();
                double head_error = get_head_error();
                double abs_error = get_absolute_error();
                bool disable_turn = fabs(abs_error) <= drivebase.track_width / 2; // prevent swivels when close to target

                // PID & Slew
                double fwd = fwd_pid.compute(fwd_error);
                double correction = head_pid.compute(head_error);

                fwd = fwd_slew.update(fwd);
                correction = head_slew.update(correction);

                // End Behavior
                if (disable_turn) {
                    head_pid.reset_integral();
                    correction = 0;
                } else {
                    // if we can still turn, don't move backwards
                    if (facing==FACE::FWD) fwd = std::max(fwd, 0.0);
                    else fwd = std::min(fwd, 0.0);
                }

                if (do_chain && fabs(fwd_error) <= chainer.range) {
                    if (fabs(fwd) < chainer.min_speed) fwd = sign(fwd) * chainer.min_speed;

                    double chain_amt = 1.0 - fabs(fwd_error) / chainer.range;
                    fwd = lerp(fwd, chainer.next_fwd, chain_amt);
                    correction = lerp(correction, chainer.next_turn, chain_amt);
                }

                // exit earlier
                if (move_type == Exit::MONO || do_chain) {
                    bool progress_condition = do_chain || signflip(fwd_error, prev_fwd_error);
                    bool in_bounds = fwd_pid.get_loose_sc().get_settling();

                    // crossed the threshold and are within a bounds
                    if (progress_condition && in_bounds) {
                        in_motion = false;
                        if (!do_chain) drivebase.brake(); //only brake if not chaining
                        return get_absolute_error();
                    }
                }

                // Scaling
                if (max_speed > 0 && fabs(fwd) > 0 && fabs(fwd) > max_speed) {
                    double ratio = max_speed / fabs(fwd);
                    fwd *= ratio;
                    correction *= ratio;
                }

                // Actuation
                drivebase.command_velocities(fwd, correction);
                last_turn = correction;
                last_fwd = fwd;

                prev_fwd_error = fwd_error;
                delay_for(pollrate);
            }

            if (!do_chain) {
                drivebase.brake();
                last_fwd = 0;
                last_turn = 0;
            }

            in_motion = false;
            return get_absolute_error();
        }

        // turn to some target heading in degrees. returns final degrees error
        // setting timeout to -1 will disable it
        // mono_move ensures motor commands are uni-directional by exiting the function once the robot overshoots and lies within settle range
        // direction can either be specified or calculated through shortest turning angle
        // chainer allows for motion chaining. range in inches, min_speed in inches/sec, and some default configuration available as well
        double turn_to(double heading_deg, ms timeout = -1, Exit move_type = Exit::LOOSE, bool relative = false, Chain chainer = Chain{}, DIR direction = DIR::EITHER) {
            const double target_radians = to_rad(heading_deg);
            const bool do_chain = chainer.range > 0;

            auto true_error = [&] {
                return calculate_turn(localizer->get_pose().dir,target_radians);
            };
            if (in_motion) return to_deg(true_error());
            in_motion = true;

            cache.set_heading(target_radians);

            bool ignore_direction = false;

            turn_pid.set_timeout(timeout);

            turn_pid.reset();
            turn_slew.ready(last_turn);

            auto get_error = [&] {

                // if we're close enough to ignore the direction
                if (turn_pid.get_loose_sc().get_settling() && !ignore_direction) ignore_direction = true;

                double error = calculate_turn(
                    localizer->get_pose().dir,
                    cache.get_heading(),
                    (ignore_direction ? DIR::EITHER : direction)
                );

                return error;
            };

            double prev_error = 0;
            while (!turn_pid.done(move_type == Exit::TIGHT) && in_motion) {

                // error calculations
                double error = get_error();

                std::cout << to_deg(error) << " " << to_deg(localizer->get_pose().dir) << std::endl;

                // pid & slew
                double turn = turn_pid.compute(error);
                double fwd = 0;

                turn = turn_slew.update(turn);

                if (do_chain && fabs(error) <= chainer.range) {
                    if (fabs(turn) < chainer.min_speed) turn = sign(turn) * chainer.min_speed;

                    double chain_amt = 1.0 - fabs(error) / chainer.range;
                    turn = lerp(turn, chainer.next_turn, chain_amt);
                    fwd = lerp(fwd, chainer.next_fwd, chain_amt);
                }

                // early exit
                if (move_type == Exit::MONO || do_chain) {
                    bool progress_condition = do_chain || signflip(error, prev_error);
                    bool in_bounds = turn_pid.get_loose_sc().get_settling();

                    if (progress_condition && in_bounds) {
                        in_motion = false;
                        if (!do_chain) drivebase.brake();
                        return to_deg(true_error());
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

            if (!do_chain) {
                drivebase.brake();
                last_turn = 0;
                last_fwd = 0;
            }

            if (relative) {
                cache.set_pos(localizer->get_pose().pos);
            }

            in_motion = false;
            return to_deg(final_error);
        }

        // turn to face some target position. returns final degrees error
        // setting timeout to -1 will disable it
        // mono_move ensures motor commands are uni-directional by exiting the function once the robot overshoots and lies within settle range
        // direction can either be specified or calculated through shortest turning angle
        // chainer allows for motion chaining. range in inches, min_speed in inches/sec, and some default configuration available as well
        double face_to(Vec target, FACE face = FACE::FWD, ms timeout = -1, Exit move_type = Exit::LOOSE, Chain chainer = Chain{}, DIR direction = DIR::EITHER) {
            Vec cur_pos = localizer->get_pose().pos;
            double facing_heading = wrap_angle(to_deg((target - cur_pos).angle()) + (face==FACE::BACK ? 180 : 0), false);
            return turn_to(facing_heading, timeout, move_type, false, chainer, direction);
        }

        double swing_to(double heading_deg, DIR direction, ms timeout = -1, Exit move_type = Exit::LOOSE) {
            const double target_radians = to_rad(heading_deg);

            auto true_error = [&] {
                return calculate_turn(localizer->get_pose().dir,target_radians);
            };
            if (in_motion) return to_deg(true_error());
            if (calculate_turn(to_rad(cache.get_heading()), to_rad(heading_deg)) == 0) return 0;
            in_motion = true;

            // fix a direction of turning first
            DIR fixed_direction = direction;
            if (direction == DIR::EITHER) fixed_direction = to_bearing(target_radians) > 0 ? DIR::LEFT : DIR::RIGHT;

            Pose last_pose = cache.get_pose();
            double turn_amount = fixed_direction == DIR::LEFT ? wrap_angle(target_radians - last_pose.dir, true) : -wrap_angle(last_pose.dir - target_radians, true);
            pros::lcd::print(2, "%f, ss %f", turn_amount, last_pose.dir);
            double arc_length = 0.5 * (2+drivebase.track_width) * fabs(turn_amount);
            Vec target_pos = last_pose.pos + Odometry::draw_arc(to_deg(last_pose.dir), arc_length, to_deg(turn_amount));
            Pose target = Pose{target_pos, target_radians};
            cache.set_pose(target);

            bool ignore_direction = false;

            swing_pid.set_timeout(timeout);

            swing_pid.reset();
            swing_slew.ready(last_turn);

            auto get_error = [&] {

                // if we're close enough to ignore the direction
                if (swing_pid.get_loose_sc().get_settling() && !ignore_direction) ignore_direction = true;

                double error = calculate_turn(
                    localizer->get_pose().dir,
                    cache.get_heading(),
                    DIR::EITHER
                );

                return error;
            };

            double prev_error = 0;
            while (!swing_pid.done(move_type == Exit::TIGHT) && in_motion) {

                // error calculations
                double error = get_error();

                // pid & slew
                double swing = swing_pid.compute(error);
                swing = swing_slew.update(swing);

                // early exit
                if (move_type == Exit::MONO) {
                    bool progress_condition = signflip(error, prev_error);
                    bool in_bounds = swing_pid.get_loose_sc().get_settling();

                    if (progress_condition && in_bounds) {
                        in_motion = false;
                        drivebase.brake();
                        return to_deg(true_error());
                    }
                }

                //actuation
                if (fixed_direction == DIR::LEFT) drivebase.command_right_only(swing, true);
                else drivebase.command_left_only(-swing, true);
                drivebase.command_brake(fixed_direction);

                prev_error = error;
                delay_for(pollrate);
            }

            double final_error = true_error();
            if (move_type != Exit::MONO) {
                drivebase.brake(true);
            }

            last_fwd = 0;
            last_turn = 0;

            in_motion = false;
            return to_deg(final_error);

        }

        /*
        double boomerang_to(Pose target, FACE dir = FACE::FWD, ms timeout = -1, Exit move_type = Exit::LOOSE, Chain chainer = Chain{}) {
            const bool do_chain = chainer.range > 0;
            if (in_motion) return mag(target.pos - localizer->get_pose().pos);
            in_motion = true;

            cache.set_pos(target);
            Pose target_pose = cache.get_pose();

            fwd_pid.set_timeout(timeout);
            head_pid.set_timeout(timeout);

            fwd_pid.reset();
            head_pid.reset();

            // support for motion chaining
            fwd_slew.ready(last_fwd);
            head_slew.ready(last_turn);
        }
         */
    };
}
