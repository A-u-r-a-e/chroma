#pragma  once
#include "api.h"
#include "chromatic/chassis/differential.hpp"
#include "chromatic/core.hpp"
#include "chromatic/core/helpers.hpp"
#include "chromatic/core/vector.hpp"
#include "chromatic/shorthands.hpp"

namespace chromatic {
    struct FieldWalls {
        double right;
        double left;
        double top;
        double bottom;
    };

    struct FieldElementBox {
        double x_min;
        double x_max;
        double y_min;
        double y_max;

        bool is_inside(Vec point) {
            bool in_x = point.x >= x_min && point.x <= x_max;
            bool in_y = point.y >= y_min && point.y <= y_max;
            return in_x && in_y;
        }

        // pose in ccw radians
        bool projection_collision(Pose pose) {

            const double x0 = pose.pos.x;
            const double y0 = pose.pos.y;

            constexpr double error_bound = 0.001;

            double dx = cos(pose.dir);
            double dy = sin(pose.dir);

            double t_min = 0.0, t_max = 1e9;

            // x check
            if (fabs(dx) <= error_bound) {
                if (x0 < x_min || x0 > x_max) return false;
            } else {
                double t1 = (x_min - x0) / dx;
                double t2 = (x_max - x0) / dx;

                if (t1 > t2) std::swap(t1, t2);

                t_min = fmax(t_min, t1);
                t_max = fmin(t_max, t2);
            }

            // y check
            if (fabs(dy) <= error_bound) {
                if (y0 < y_min || y0 > y_max) return false;
            } else {
                double t1 = (y_min - y0) / dy;
                double t2 = (y_max - y0) / dy;

                if (t1 > t2) std::swap(t1, t2);

                t_min = fmax(t_min, t1);
                t_max = fmin(t_max, t2);
            }

            return t_min < t_max;
        }
    };
}
