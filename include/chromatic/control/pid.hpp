#pragma once
#include "chromatic/shorthands.hpp"
#include "chromatic/core.hpp"
#include "chromatic/control/settle_conditions.hpp"

namespace chromatic {

    struct PID {
    private:
        const double kP, kI, kD, integral_range; //  PID coefficients and the start_integral range
        const double max_accum, max_damp, max_output; // maximum integral/dampening/total output value contribution

        ms timeout; // timeout that can be set on demand, is default disabled with -1, max time elapsed before done = true, this does not affect compute

        SettleCondition sc_tight, sc_loose; // tight and loose settle conditions, note that this does not affect compute output

        double sum_error, prev_error; // integral term (without multiplier), and last call error
        ms sum_time, prev_time, begin_time;; // time elapsed, and last compute call time

        bool fresh, overshot; // compute call being right after reset and if we have reached threshold (kill integral)

    public:

        // pid constants, with two settle conditions
        PID(
            double kP, double kI, double kD, double integral_range,
            SettleCondition sc_tight, SettleCondition sc_loose,
            double max_accum, double max_damp, double max_output
        ):
            kP{kP}, kI{kI}, kD{kD}, integral_range{integral_range},
            sc_tight{sc_tight}, sc_loose{sc_loose},
            max_accum{max_accum}, max_damp{max_damp}, max_output{max_output}
        {
            prev_error = 0;
            prev_time = now();
            begin_time = now();
            timeout = -1;
            reset();
        }

        // checks if the pid is settled in either loose or tight
        inline bool loose() const {
            return (sc_loose() || sc_tight());
        }

        inline bool tight() const {
            return (sc_tight());
        }

        // checks if the pid has either settled or timed out (if set)
        inline bool done(bool strict = false) const {
            bool failsafe_activated = (timeout >= 0 && sum_time > 0 && time_left() >= timeout);
            return (failsafe_activated || (strict ? tight() : loose()));
        }

        // check time left in current run, guaranteed to be >= 0
        inline ms time_left() const {
            return std::max(static_cast<uint32_t>(now() - begin_time), static_cast<uint32_t>(0));
        }

        // gets the loose settle condition object
        inline SettleCondition get_loose_sc() const {
            return sc_loose;
        }

        // gets the tight settle condition object
        inline SettleCondition get_tight_sc() const {
            return sc_tight;
        }

        // set to -1 to disable timeout (ms)
        inline void set_timeout(ms timeout) {
            this->timeout = timeout;
        }

        // reset integral
        inline void reset_integral() {
            sum_error = 0;
        }

        // reset the pid for a new motion or target
        inline void reset() {
            sum_error = 0;
            sum_time = 0;
            sc_loose.reset();
            sc_tight.reset();
            overshot = false;
            fresh = true;
            // do not reset prev_s because that is pointless and only creates more garbage values (reset() != ready())
        }

        // computes pid output, note that this does not work for moving targets
        inline double compute(double error) {
            if (fresh) {
                prev_error = error;
                prev_time = now();
                begin_time = now();
            }
            fresh = false;

            ms this_time = now();
            double dt = (this_time - prev_time) / 1000.0;

            sc_loose.update(error, this_time);
            sc_tight.update(error, this_time);

            // reset integral after overshooting
            // overshot = signflip(prev_error, error);
            // disable integral after overshooting
            if (signflip(prev_error, error)) overshot = true;

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
            if (signflip(P, D)) { // this is to ensure P term is not dead
                D = std::clamp(D, -fabs(P), fabs(P));
            }


            double output = P + I + D;
            output = std::clamp(output, -max_output, max_output);

            prev_error = error;
            sum_time += dt * 1000;
            prev_time = this_time;

            return output;
        }

    };
}
