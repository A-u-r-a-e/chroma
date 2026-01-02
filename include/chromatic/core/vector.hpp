#pragma once
#include "std.hpp"

namespace chromatic {

    struct Vec{
        double x;
        double y;

        /**
         * @brief Construct a new Vec object
         *
         * @param x x value
         * @param y y value
         */
        constexpr Vec(double x = 0, double y = 0) : x(x), y(y) {}

        /**
         * @brief Create a Vec from anti-clockwise polar coordinates
         *
         * @param radians radians measure from i basis vector
         * @param magnitude length in said direction
         * @return Vec
         */
        inline static Vec Polar(double radians, double magnitude = 1) {
            return Vec(cos(radians) * magnitude, sin(radians) * magnitude);
        }

        /**
         * @brief Returns the magnitude of the Vec
         *
         * @return double
         */
        inline double mag() const {return sqrt(pow(x,2)+pow(y,2));}

        /**
         * @brief Returns the angle (radians) of the vector from (0, 2pi)
         *
         * @return double
         */
        inline double angle() const {return atan2(this->y, this->x);}

        /**
         * @brief If this vector is a unit vector
         *
         * @return true
         * @return false
         */
        inline bool is_unit() const {return this->mag() == 1;}

        /**
         * @brief The unit direction vector if it exists
         *
         * @return Vec
         */
        inline Vec norm() const {
            double mag = this->mag();
            if (mag == 0) return *this;
            return Vec{this->x/mag, this->y/mag};
        }

        /**
         * @brief Comparators
         *
         * @param other vector to compare to
         */
        inline bool operator==(const Vec other) const {return (this->x == other.x && this->y == other.y);}
        inline bool operator!=(const Vec other) const {return (this->x != other.x || this->y != other.y);}

        /**
         * @brief Element-wise operations for vector-on-vector
         *
         * @param other The other vector
         * @return Vec
         */
        inline Vec operator+(const Vec other) const {return Vec(this->x+other.x, this->y+other.y);}
        inline Vec operator-(const Vec other) const {return Vec(this->x-other.x, this->y-other.y);}

        /**
         * @brief Scalar multiplication and division
         *
         * @param other The scalar
         * @return Vec
         */
        inline Vec operator*(const double other) const {return Vec(this->x * other, this->y * other);}
        inline Vec operator/(const double other) const {return Vec(this->x/other, this->y/other);}

        // Assignment operator MUST be default for atomic expressions
    };

    /**
     * @brief magnitude of a vector
     *
     * @param a the vector
     * @return double
     */
    inline double mag(const Vec v) {return v.mag();}

    /**
     * @brief Finds the unit vector
     *
     * @param a
     * @return Vec
     */
    inline Vec norm(const Vec v) {return v.norm();}

    /**
     * @brief Performs the dot product
     *
     * @param a a vector
     * @param b the other vector
     * @return double
     */
    inline double dot(const Vec v, const Vec w) {return v.x * w.x + v.y * w.y;}

    /**
     * @brief Performs the cross product or determinant of the 2x2 matrix formed by these two vectors
     * @note negative if b is counter-clock rel a, positive if clock rel a
     *
     * @param a column 1 vector
     * @param b column 2 vector
     * @return double
     */
    inline double cross(const Vec v, const Vec w) {return v.x * w.y - v.y * w.x;}

    /**
     * @brief Get the result of an anti-clockwise rotation of a vector
     *
     * @param v the vector to be rotated
     * @param a radians to be rotated anti-clockwise
     * @return Vec
     */
    inline Vec rotate(const Vec v, const double a) {
        Vec out{
            v.x * cos(a) - v.y * sin(a),
            v.x * sin(a) + v.y * cos(a)
        };
        return out;
    }

    /**
     * @brief lerp but for vectors
     *
     * @param a first point
     * @param b second point
     * @param t decimal from [0, 1] indicating progress between a and b
     * @return Vec
     */
    inline Vec lerp(const Vec a, const Vec b, double t) {return a * (1 - t) + b * t;}

    constexpr inline Vec ZeroVec{0, 0};

}
