#pragma once
#include "vector.hpp"

namespace chromatic {
    struct Pose {
        Vec pos;
        double dir;

        // a pose based on position and direction
        Pose(
            Vec pos = ZeroVec, double dir = 0
        ):
            pos{pos}, dir{dir}
        {}

        // gets itself
        inline Pose pose() const {
            return *this;
        }

        // raycast a pose using its direction by some amount
        static inline Pose project(const Pose& source, double amount) {
            Pose result = Pose(source.pos + Vec::Polar(source.dir, amount), source.dir);
            return result;
        }
    };

    struct PoseV : public Pose {
        Vec vel;
        double turn;

        // pose but with velocities of both rotational and translational
        PoseV(
            Vec pos = ZeroVec, double dir = 0, Vec vel = ZeroVec, double turn = 0
        ):
            Pose{pos, dir}, vel{vel}, turn{turn}
        {}

        // gets the pose of a posev
        inline Pose pose() const {
            return Pose{pos, dir};
        }

        // gets itself
        inline PoseV posev() const {
            return *this;
        }

        // gets a poseV from a pose
        static inline PoseV fromPose(const Pose& pose) {
            return PoseV(pose.pos, pose.dir, ZeroVec, 0);
        }
    };


}
