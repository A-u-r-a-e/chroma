#pragma once
#include "api.h"
#include "chromatic/core.hpp"
#include "chromatic/shorthands.hpp"
#include "chromatic/chassis/tof.hpp"

namespace chromatic {
    struct SingleTOF {
    private:
        TOF &sensor;
        double offset; // directed, in inches!
    public:
        // offset is positive as you move to the back
        SingleTOF(TOF& sensor, double fwd_offset) : sensor{sensor}, offset{fwd_offset} {

        }

        // find the direct distance
        // will return -1 if angle is too big/invalid
        double find_norm_distance(double offset_deg_ccw) {
            if (fabs(offset_deg_ccw) >= 90) return -1;
            double theta_rad = to_rad(offset_deg_ccw);

            double true_dist = sensor.get_top();
            if (true_dist < 0) return -1;

            true_dist += tan(theta_rad) * offset;
            double from_wall = true_dist * 1.0 / cos(theta_rad);

            return from_wall;
        }

        void update() {
            sensor.update();
        }

        double get_offset() {
            return offset;
        }
    };

    struct DoubleTOF {
    private:
        TOF &left, &right;
        double left_offset, right_offset;
    public:
        DoubleTOF(TOF& left_sensor, TOF& right_sensor, double left_offset, double right_offset):
            left{left_sensor}, right{right_sensor}, left_offset{left_offset}, right_offset{right_offset}
        {

        }

        // gets the relative angle from perpendicular (l=r), -1 if reading errors
        double find_offset_deg_ccw() {
            double l_raw = left.get_top();
            double r_raw = right.get_top();
            if (l_raw < 0 || r_raw < 0) return -1;

            double sensor_difference = l_raw - r_raw;
            double sensor_width = left_offset + right_offset;

            double theta_rad = atan2(sensor_difference, sensor_width);
            double offset_deg_ccw = to_deg(theta_rad);

            return offset_deg_ccw;
        }

        double find_norm_distance(double offset_deg_ccw) {
            if (fabs(offset_deg_ccw) >= 90) return -1;
            double theta_rad = to_rad(offset_deg_ccw);

            double l_read = left.get_denoised();
            double r_read = right.get_denoised();
            if (l_read < 0 || r_read < 0) return -1;

            double sensor_width = left_offset + right_offset;
            double true_dist = lerp(l_read, r_read, left_offset / sensor_width);

            double from_wall = true_dist / cos(theta_rad);

            return from_wall;
        }

        void update() {
            left.update();
            right.update();
        }

        double get_left_offset() {
            return left_offset;
        }

        double get_right_offset() {
            return right_offset;
        }
    };
}
