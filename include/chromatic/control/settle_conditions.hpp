#pragma once
#include "chromatic/shorthands.hpp"
#include "chromatic/core.hpp"

namespace chromatic {
    struct SettleCondition {
    private:
        const double settle_range;
        const ms settle_duration;

        ms settling_since;
        ms last_update;
        bool settling;
    public:

        // settle condition
        SettleCondition(
            double settle_range, ms settle_duration
        ):
            settle_range(settle_range), settle_duration(settle_duration)
        {
            settling_since = -1;
            settling = false;
            last_update = now();

        }

        // evaluate if settled (based on updates and stuff)
        inline bool operator()() const {
            return (settling && settling_since > 0 && last_update - settling_since >= settle_duration);
        }

        // gets if the error was in range last update
        inline bool get_settling() const {
            return settling;
        }

        // update the settle condition with a error and an optionally provided time
        inline void update(double error, ms cur_time = now()) {
            bool in_range = fabs(error) < settle_range;

            if (!settling && in_range) settling_since = cur_time;
            if (!in_range) settling_since = -1;

            settling = in_range;
            last_update = cur_time;
        }

        // reset the settle condition for a new motion
        inline void reset() {
            settling_since = -1;
            settling = false;
            last_update = now();
        }
    };
}
