#pragma once
#include "chromatic/shorthands.hpp"
#include "chromatic/core.hpp"

struct PID {
private:
    const double kP, kI, kD;
    const double settle_error, start_integral;
    const ms settle_time;

    double last_error, sum_error;
public:
    PID(
        double kP, double kI, double kD, double settle_error, ms settle_time, double start_integral = 0):
        kP(kP), kI(kI), kD(kD), settle_error(settle_error), settle_time(settle_time), start_integral(start_integral)
    {

    }

    double compute(double error, double delta_error = 0) {
        double e = error;
        // if our error changed, and d_e is 0, that means we ommitted it, so we calculate de
        // if our error changed, and d_e isnt 0, we override de
        // iff our error didnt change, and d_e is 0, then we use it
        // if our error didnt change, and d_e isnt 0, then we use it
        // essentially, only if our error changed but d_e is 0 do we have a special condition to calculate de
        double de = (last_error != error && delta_error == 0 ? error - last_error : delta_error);
        double inte = sum_error + error;
    }

    void reset() {}

    bool settling() const {}

    bool settled() const {}

};
