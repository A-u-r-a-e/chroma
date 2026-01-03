#pragma once
#include "chromatic/shorthands.hpp"
#include "chromatic/core.hpp"

namespace chromatic {
    struct SettleCondition {
    private:
        const double settle_range;
        const ms settle_duration;

        ms settling_since;
        bool settling;
    public:

        SettleCondition(double settle_range, ms settle_duration) : settle_range(settle_range), settle_duration(settle_duration) {
            settling_since = INT32_MAX;
            settling = false;
        }

        inline bool operator()() {
            return (settling && now() - settling_since >= settle_duration);
        }

        inline void update(double error, ms cur_time = now()) {
            bool in_range = fabs(error) < settle_range;

            if (!settling && in_range) settling_since = cur_time;
            if (!in_range) settling_since = INT32_MAX;

            settling = in_range;
        }

        inline void reset() {
            settling_since = INT32_MAX;
            settling = false;
        }
    };
}
