#pragma once
#include "chromatic/core/helpers.hpp"
#include "chromatic/shorthands.hpp"
#include "chromatic/core.hpp"
#include "chromatic/chassis.hpp"
#include "chromatic/control/pid.hpp"

namespace chromatic {

    struct MotionController {
    private:
        const Differential &drivetrain;
        const EncodersIMU &localizer;
        PID linear_pid, turn_pid;
        std::atomic<bool> in_motion;

    public:

        MotionController(
            const Differential &drivetrain, const EncodersIMU &localizer, const PID linear_pid, const PID turn_pid):
            drivetrain{drivetrain}, localizer{localizer}, linear_pid{linear_pid}, turn_pid{turn_pid}
        {
            in_motion = false;
        }

        // if MotionController is active or not
        bool is_in_motion() {
            return in_motion;
        }

        // drives relative to the current heading by some amount command, returns true if successful
        bool straight_drive(double amount, bool ensure_facing = false) {
            if (in_motion) return false;
            in_motion = true;

            Pose pre_motion = localizer.get_pose();
            Pose target_pose = Pose(pre_motion.pos + Vec::Polar(pre_motion.dir, amount), pre_motion.dir);

            auto get_error = [&] {
                double distance = mag(target_pose.pos - localizer.get_pose().pos);
                return distance;
            };

            auto get_angular_error = [&] {
                Pose cur_pose = localizer.get_pose();
                double angle_offset = calculate_turn(
                    cur_pose.dir,
                    (target_pose.pos - cur_pose.pos).angle()
                );
                return angle_offset;
            };


            while (!linear_pid.settled() && in_motion) {
                // mind the signs
                // you might want to set target to 0 so that you feed in negatives values to pid so that the output is positive
            }

            while ((!ensure_facing || turn_pid.settled()) && in_motion) {

            }

        }


    };



}
