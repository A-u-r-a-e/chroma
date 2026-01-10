#pragma once
#include "std.hpp"
#include "enums.hpp"

namespace chromatic {

    // {%} Trig Helpers

    /**
     * @brief sinx/x with LIMIT
     *
     * @param x value to perform sinx/x on, rad
     * @return double
     */
    inline double sinc(double x) {
        return (fabs(x) <= 1e-8 ? 1 : sin(x)/x);
    }
    /**
     * @brief (1-cosx)/x with LIMIT
     *
     * @param x rad
     * @return double lim
     */
    inline double cosc(double x) {
        return (fabs(x) <= 1e-8 ? 0 : (1 - cos(x)) / x);
    }

    /**
     * @brief Conversion from radians to degrees
     *
     * @param radians radians value
     * @return double
     */
    inline double to_deg(double radians) {
        return radians * (180 / PI);
    }
    /**
     * @brief Conversion from degrees to radians
     *
     * @param degrees degrees value
     * @return double
     */
    inline double to_rad(double degrees) {
        return degrees * (PI / 180);
    }

    /**
     * @brief Convert from a turn to a heading
     *
     * @param turn [-pi or 180, pi or 180]
     * @param radians is this in radians or not
     * @return double [0, 2pi or 360]
     */
    inline double to_head(double turn, bool radians = true) {
        double compare = (radians ? PI : 180);
        return turn + compare;
    }
    /**
     * @brief Convert from a heading to a turn
     *
     * @param turn [0, 2pi or 360]
     * @param radians is this in radians or not
     * @return double [-pi or 180, pi or 180]
     */
    inline double to_turn(double heading, bool radians = true) {
        double compare = (radians ? PI : 180);
        if (heading >= compare) return heading - 2 * compare;
        return heading;
    }

    /**
     * @brief Normalize angle for heading
     *
     * @param theta the value
     * @param radians the range of [0, div] to modulo this into
     * @return double
     */
    inline double norm_angle(double theta, bool radians = true) {
        double div = (radians ? 2 * PI : 360);
        return fmod(fmod(theta+div,div)+div, div);
    }

    /**
     * @brief Find the turning angle needed to achieve end from start. Radians only.
     * @note assumes ccw radians
     *
     * @param start Initial radians
     * @param end Final Radians
     * @param way Turning Direction, blank for EITHER/optimal
     * @return double
     */
    inline double calculate_turn(double start, double end, DIR way = DIR::EITHER) {
        switch (way) {
        case DIR::CLOCKWISE:
            return 2*PI - norm_angle(start - end);
            break;
        case DIR::COUNTERCLOCKWISE:
            return norm_angle(end - start);
            break;
        default:
            return to_turn(norm_angle(end - start));
        }
    }

    // {%} Numerical Helpers

    /**
     * @brief Find the sign (including zeros) of a number
     *
     * @param value the number
     * @return SIGN
     */
    inline SIGN as_SIGN(double value) {
        if (value == 0) return SIGN::ZERO;
        return (value > 0 ? SIGN::POSITIVE : SIGN::NEGATIVE);
    }

    /**
     * @brief average of a vector of doubles
     *
     * @param nums vector of doubles
     * @return double
     */
    inline double average(const std::vector<double>& nums) {
        if (nums.size()==0) { return 0; }
        double sum=0;
        for (int i = 0; i < static_cast<int>(nums.size()); i ++) { sum += nums[i]; }
        return sum/nums.size();
    }
    inline double naverage(const std::vector<int>& nums) {
        if (nums.size()==0) { return 0; }
        int sum=0;
        for (int i = 0; i < static_cast<int>(nums.size()); i ++) { sum += nums[i]; }
        return static_cast<double>(sum)/nums.size();
    }

    /**
     * @brief see if a value has switched signs or not
     *
     * @param a last value
     * @param b current value
     * @return true
     * @return false
     */
    inline bool signflip(double a, double b) {
        return a * b < 0;
    }

    /**
     * @brief Use for denominators that may be zero
     *
     * @param x the denom
     * @return double the safe denom that isn't zero
     *
     * @note this is a lie this is quadratic smooth relu
     */
    inline double nozero(double x) {
        return std::copysign(sqrt(pow(x, 2) + 0.0001), x);
    }

}
