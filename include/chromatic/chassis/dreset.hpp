#pragma once
#include "api.h"
#include "chromatic/core.hpp"
#include "chromatic/shorthands.hpp"
#include "chromatic/chassis/tof.hpp"

namespace chromatic {
    struct SingleTOF {
    private:
        TOF& sensor;
        double offset; // directed, in inches!
    public:
        SingleTOF(TOF& sensor, double offset) : sensor{sensor}, offset{offset} {

        }


    };

    struct DoubleTOF {
    private:
        SingleTOF left;
        SingleTOF right;
    public:
        DoubleTOF(TOF& left_sensor, TOF& right_sensor, double left_offset, double right_offset):
            left{left_sensor, left_offset},
            right{right_sensor, right_offset}
        {

        }
    };
}
