#pragma once
#include "chromatic/shorthands.hpp"
#include "chromatic/core.hpp"

namespace chromatic {
    struct SlewRate {
    private:
        double acc_limit;

        double last_slew;
        ms last_update;
    public:

        // set a 0 acc_limit for no limit
        SlewRate(
            double acc_limit = 0
        ):
            acc_limit(acc_limit)
        {
            last_slew = 0;
            last_update = now();
        }

        // reset slew limit
        void ready(double prev = 0) {
            last_slew = prev;
            last_update = now();
        }

        // set new slew limit
        void set_limit(double new_limit) {
            acc_limit = new_limit;
        }

        // update desired output
        double update(double desired_vel) {
            if (acc_limit == 0) {
                last_slew = desired_vel;
                return desired_vel;
            }
            double dt = (now() - last_update) / 1000.0;
            double change_limit = acc_limit * dt;
            double desired_change = desired_vel - last_slew;
            double slewed = last_slew + std::clamp(desired_change, -change_limit, change_limit);
            last_slew = slewed;
            return slewed;
        }
    };
}
