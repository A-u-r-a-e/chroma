#pragma  once
#include "api.h"
#include "chromatic/chassis/differential.hpp"
#include "chromatic/core.hpp"
#include "chromatic/core/vector.hpp"
#include "chromatic/shorthands.hpp"
#include <cmath>

namespace chromatic {
    struct Odometry {
    protected:
        std::atomic<bool> active{false};

        mutable pros::MutexVar<PoseV> cur_posev{ZeroVec, 0, ZeroVec, 0};

        //imu but radians ong, also converts to ccw
        double get_imu_rad(pros::IMU &imu) {
            return to_rad(360-imu.get_heading());
        }

        // theta radians turning, ccw
        Vec calculate_arc(Vec d_position, double d_theta) {
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
        void compute(Vec dpos, double dang, ms dt) {
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
            set_posev(cur);
        }

    public:

        virtual void calibrate() = 0;
        virtual void localize(ms poll_delay) = 0;

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
    };

    struct EncodersIMU : public Odometry {
    private:
        Differential& drivebase;
        pros::IMU& inertial;

        double last_ang;
        double last_lin;
        ms last_time;

        std::atomic<bool> calibrated;
    public:

        EncodersIMU(
            Differential &drivebase, pros::IMU &inertial):
            drivebase(drivebase), inertial(inertial)
        {
            calibrated = false;
            last_ang = 0;
            last_lin = 0;
            last_time = now();
        }

        // calibrate odom oand reset drivebase
        void calibrate() override {
            active = false;
            calibrated = false;
            inertial.reset(true);
            inertial.tare();
            drivebase.reset();
            last_ang = 0;
            last_lin = 0;
            last_time = now();
            calibrated = true;
        }

        // continuously calculate pose and state
        void localize(ms poll_delay = 10) override {
            while (!calibrated);
            active = true;
            while (active && calibrated) {
                double ang = get_imu_rad(inertial);
                double lin = (2 * PI * drivebase.wheel_radius) * (average(drivebase.left_mg.get_position_all()) + average(drivebase.right_mg.get_position_all()))/(2 * drivebase.get_ticks_per_wheel_rev());

                Vec dpos{lin - last_lin, 0};
                double dang = ang - last_ang;
                double dt = now() - last_time;

                if (dt > 0) {
                    compute(dpos,dang, dt);
                    last_ang = ang;
                    last_lin = lin;
                    last_time = now();
                }
                pros::lcd::print(0, "(%f, %f), %f", this->get_pose().pos.x, this->get_pose().pos.y, to_deg(this->get_pose().dir));

                delay_for(poll_delay);
            }
        }
    };

}
