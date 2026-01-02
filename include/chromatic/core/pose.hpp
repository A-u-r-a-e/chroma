#pragma once
#include "vector.hpp"

namespace chromatic {
    struct Pose {
        Vec pos;
        double dir;
        /**
         * @brief Pose is a instaneous snapshot of the position of a robot's state
         *
         * @param pos location
         * @param dir facing direction
         */
        Pose(Vec pos, double dir) : pos{pos}, dir{dir} {}

        /**
         * @brief Return itself
         *
         * @return Pose
         */
        inline Pose pose() const {return *this;}
    };

    struct PoseV : public Pose {
        Vec vel;
        double turn;
        /**
         * @brief PoseV is the extension of Pose to feature velocities as well
         *
         * @param pos location
         * @param dir facing direction
         * @param vel rate of change in location
         * @param turn rate of change in direction
         */
        PoseV(Vec pos, double dir, Vec vel, double turn) : Pose{pos, dir}, vel{vel}, turn{turn} {}

        /**
         * @brief Reduce to a Pose
         *
         * @return Pose
         */
        inline Pose pose() const {return Pose{pos, dir};}

        /**
         * @brief Return itself
         *
         * @return PoseV
         */
        inline PoseV posev() const {return *this;}
    };
}
