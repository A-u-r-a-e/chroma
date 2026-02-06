#pragma once
#include "chromatic/core.hpp"

namespace chromatic {
    struct CubicSpline{
    private:
        /*
         * The Hermite Basis with Nth derivatives
         *
         * t: at time
         * d: dth derivative
         * term: for each term in the summation
         */
        inline double basis(double t, int d, int term) const {
            switch (d) {
                case 0:
                    switch (term) {
                        case 0: return 2 * pow(t, 3) - 3 * pow(t,2)+1;
                        case 1: return pow(t,3) - 2 * pow(t,2) + t;
                        case 2: return (-2) * pow(t,3) + 3 * pow(t,2);
                        case 3: return pow(t,3) - pow(t,2);
                        default: return 0.0;
                    }
                case 1:
                    switch (term) {
                        case 0: return 6 * pow(t,2) - 6*t;
                        case 1: return 3 * pow(t,2) - 4*t + 1;
                        case 2: return (-6) * pow(t,2) + 6*t;
                        case 3: return 3 * pow(t,2) - 2*t;
                        default: return 0.0;
                    }
                case 2:
                    switch (term) {
                        case 0: return 12 * t - 6;
                        case 1: return 6 * t - 4;
                        case 2: return (-12) * t + 6;
                        case 3: return 6 * t - 2;
                        default: return 0.0;
                    }
                default: return 0.0;
            }
        }
    public:

        Vec p0; Vec v0; Vec p1; Vec v1;

        [[deprecated("Construct with CubicSpline::Hermite() instead")]]
        CubicSpline(Vec p0, Vec v0, Vec p1, Vec v1) : p0(p0), v0(v0), p1(p1), v1(v1) {};

        /*
         * Creates a Cubic Spline using Bezier control points
         *
         * p0: first control point
         * p1: second control point
         * p2: third control point
         * p3: fourth control point
         */
        inline static CubicSpline Bezier(Vec p0, Vec p1, Vec p2, Vec p3) {
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return CubicSpline(p0, (p1 - p0) * 3, p3, (p3 - p2) * 3);
            #pragma GCC diagnostic pop
        }

        /*
         * Creates a Cubic Spline using Hermite control points
         *
         * p0: Initial position
         * v0: Initial velocity
         * p1: Final position
         * v1: Final velocity
         * @return CubicSpline
         */
        inline static CubicSpline Hermite(Vec p0, Vec v0, Vec p1, Vec v1) {
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return CubicSpline(p0, v0, p1, v1);
            #pragma GCC diagnostic pop
        }

        /*
         * Evalutes the position/velocity/acceleration on the spline at time t
         *
         * t: [0, 1] The time point on the spline to calculate for.
         * d: the dth derivative value, so [0,1,2] for [pos,vel,acc]
         *
         * returns - Corresponding vector coordinate at said given time point
         */
        inline Vec evaluate(double t, int d = 0) const {return (p0 * basis(t, d, 0) + v0 * basis(t, d, 1) + p1 * basis(t, d, 2) + v1 * basis(t, d, 3));}

        /*
         * Calculates curvature of a hermite spline at a time point
         *
         * t: [0, 1] time point
         *
         * returns - negative means clockwise
         */
        inline double curvature(double t) const {
            Vec vel = evaluate(t, 1); Vec acc = evaluate(t, 2);
            if (vel.mag() == 0) return 0;
            return ((vel.x * acc.y - vel.y * acc.x) / pow((vel).mag(), 3));
        }

        /*
         * Calculates angular velocity at point using curvature and velocity
         *
         * t: [0, 1] time point
         */
        inline double angular_velocity(double t) const {
            double linear_velocity = (evaluate(t, 1)).mag();
            double signed_curvature = curvature(t);
            return linear_velocity * signed_curvature;
        }

        /*
         * Approximates arc length numerically
         *
         * subdivisions: how many equal-time segments to chunk this into
         */
        inline double arc_length(int subdivisions = 1000) const {
            double S = 0;
            for (double i = 0; i < subdivisions; i ++) S += mag(this->evaluate((i+1) / subdivisions)  - this->evaluate((i) / subdivisions));
            return S;
        }
    };
}
