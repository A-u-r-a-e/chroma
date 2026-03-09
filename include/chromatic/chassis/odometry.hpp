#pragma  once
#include "api.h"
#include "chromatic/chassis/differential.hpp"
#include "chromatic/core.hpp"
#include "chromatic/core/helpers.hpp"
#include "chromatic/core/vector.hpp"
#include "chromatic/shorthands.hpp"

namespace chromatic {

    struct Odometry {
    protected:
        std::atomic<bool> active{false};

        mutable pros::MutexVar<PoseV> cur_posev{ZeroVec, 0, ZeroVec, 0};

        //imu but radians ong, also converts to ccw
        double get_imu_rad(pros::IMU &imu) {
            return to_rad(360-imu.get_heading());
        }

        // update the current state (pose and velocities)
        void update(PoseV posev) {
            *(cur_posev.lock()) = posev;
            return;
        }

        // theta radians turning, ccw
        static Vec calculate_arc(Vec d_position, double d_theta) {
            Vec x_transform{
                d_position.x * sinc(d_theta),
                d_position.x * cosc(d_theta)
            };
            Vec y_transform{
                -d_position.y * cosc(d_theta),
                d_position.y * sinc(d_theta)
            };

            return x_transform + y_transform;
        }

        // calculate pose
        void integrate(Vec dpos, double dang, ms dt) {
            PoseV last_pose = get_posev();
            double ds = dt / 1000.0; // dt in seconds
            dpos = rotate(dpos, last_pose.dir);
            Vec delta = calculate_arc(dpos, dang);
            PoseV cur{
                last_pose.pos + delta,
                wrap_angle(last_pose.dir + dang),
                delta / ds,
                dang / ds
            };
            update(cur);
        }

    public:

        virtual void calibrate() = 0;
        virtual void localize(ms pollrate) = 0;

        // public facing call to arc drawer
        static Vec draw_arc(double start_head_deg, double delta_pos_in, double delta_head_deg) {
            Vec dpos = Vec::Polar(to_rad(start_head_deg), delta_pos_in);
            double dang = to_rad(delta_head_deg);

            Vec post_arc = calculate_arc(dpos, dang);

            return post_arc;
        }

        // force set the current state (pose and velocities)
        void set_posev(PoseV posev) {
            *(cur_posev.lock()) = posev;
            return;
        }

        // force set the current pose
        void set_pose(Pose pose) {
            auto posev = cur_posev.lock();
            posev->pos = pose.pos; posev->dir = pose.dir;
            return;
        }

        // get odometry state (pose and velocities) readings, angle is ccw
        PoseV get_posev() const {
            auto posev = cur_posev.lock()->posev();
            return posev;
        }

        // get odometry pose readings, angle is ccw
        Pose get_pose() const {
            auto pose = cur_posev.lock()->pose();
            return pose;
        }

        // stop odometry update loop
        void stop_loop() {
            active = false;
        }

        // override the x value
        void override_x(double new_x) {
            Pose cur = this->get_pose();
            cur.pos.x = new_x;
            this->set_pose(cur);
        }

        // override the yvalue
        void override_y(double new_y) {
            Pose cur = this->get_pose();
            cur.pos.y = new_y;
            this->set_pose(cur);
        }
    };
}
