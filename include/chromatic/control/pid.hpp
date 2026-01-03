#pragma once
#include "chromatic/shorthands.hpp"
#include "chromatic/core.hpp"
#include "chromatic/control/settle_conditions.hpp"

namespace chromatic {

    struct PID {
    private:
        const double kP, kI, kD, integral_range; //  PID coefficients and the start_integral range
        const ms failsafe; // max time elapsed before done = true, note that this does not affect compute output
        const double max_accum, max_damp, max_output; // maximum integral/dampening/total output value contribution

        SettleCondition sc_tight, sc_loose; // tight and loose settle conditions, note that this does not affect compute output

        double sum_error, prev_error; // integral term (without multiplier), and last call error
        ms sum_time, prev_time; // time elapsed, and last compute call time

        bool fresh, overshot; // compute call being right after reset and if we have reached threshold (kill integral)

    public:

        PID(
            double kP, double kI, double kD, double integral_range,
            SettleCondition sc_tight, SettleCondition sc_loose, ms failsafe,
            double max_accum, double max_damp, double max_output
        ):
            kP{kP}, kI{kI}, kD{kD}, integral_range{integral_range},
            sc_tight{sc_tight}, sc_loose{sc_loose}, failsafe{failsafe},
            max_accum{max_accum}, max_damp{max_damp}, max_output{max_output}
        {
            prev_error = 0;
            prev_time = now();
            reset();
        }

        inline bool settled() {
            return (sc_loose() || sc_tight());
        }

        inline bool done() {
            return (sum_time >= failsafe || settled());
        }

        inline void reset() {
            sum_error = 0;
            sum_time = 0;
            sc_loose.reset();
            sc_tight.reset();
            overshot = false;
            fresh = true;
        }

        inline double compute(double error) {
            ms this_time = now();
            ms dt = this_time - prev_time;
            if (fresh) {
                prev_error = error;
                prev_time = this_time;
            }
            fresh = false;

            sc_loose.update(error, this_time);
            sc_tight.update(error, this_time);

            // reset integral after overshooting
            /*overshot = prev_error * error < 0;*/
            // disable integral after overshooting
            if (prev_error * error < 0) overshot = true;

            double P{0}, I{0}, D{0};

            // Proportional operations
            P = kP * error;

            // Integral operations
            if (overshot || fabs(error) > integral_range) {
                sum_error = 0;
            } else {
                sum_error += error * dt;
                I = std::clamp(kI * sum_error, -max_accum, max_accum);
            }

            // Derivative operations
            if (dt > 0) {
                D = (error - prev_error) / dt;
                D = std::clamp(kD * D, -max_damp, max_damp);
            }

            double output = P + I + D;
            output = std::clamp(output, -max_output, max_output);

            prev_error = error;
            sum_time += dt;
            prev_time = this_time;

            return output;
        }

    };
}
