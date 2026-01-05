#pragma  once
#include "api.h"
#include "chromatic/chassis/differential.hpp"
#include "chromatic/core.hpp"
#include "chromatic/core/vector.hpp"
#include "chromatic/shorthands.hpp"

namespace chromatic {
    struct Odometry {
    protected:
        std::atomic<bool> active{false};

        mutable pros::MutexVar<PoseV> cur_posev{ZeroVec, 0, ZeroVec, 0};

        //imu but radians ong, also converts to ccw
        double get_imu_rad(pros::IMU imu) {
            return to_rad(-imu.get_rotation());
        }

        // theta radians turning, ccw
        Vec calculate_arc(Vec direction, double theta) {
            Vec x_transform{
                direction.x * sinc(theta),
                direction.x * cosc(theta)
            };
            Vec y_transform{
                -direction.y * cosc(theta),
                direction.y * sinc(theta)
            };

            return x_transform + y_transform;
        }

    public:

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
        const Differential& drivetrain;
        const pros::IMU& inertial;
        const double ticks_per_rotation;

        double last_ang;
        double last_lin;
        ms last_time;
    public:

        // btw ticks per rotation is the encoder ticks of the encoder per full 360º rotation
        EncodersIMU(
            Differential &drivetrain, pros::IMU &inertial, double ticks_per_rotation):
            drivetrain(drivetrain), inertial(inertial), ticks_per_rotation(ticks_per_rotation)
        {}

        // calibrate odometry and set starting pose
        void calibrate(Pose init) {
            active = false;
            inertial.reset();
            set_pose(init);
            last_ang = get_imu_rad(inertial);
            last_lin = (2 * PI * drivetrain.wheel_radius) *
                (average(drivetrain.left_mg.get_position_all()) + average(drivetrain.right_mg.get_position_all()))/(2 * ticks_per_rotation);
            last_time = now();
        }

        // calculate pose
        void compute(double dlin, double dang, ms dt) {
            double ds = dt / 1000.0; // dt in seconds
            Vec delta = calculate_arc(Vec::Polar(last_ang, dlin), dang);
            PoseV cur = get_posev();
            cur.pos = cur.pos + delta;
            cur.vel = delta / ds;
            cur.dir = get_imu_rad(inertial);
            cur.turn = dang / ds;
            set_posev(cur);
        }

        // continuously calculate pose and state
        void localize(ms poll_delay = 10) {
            active = true;
            while (active) {
                double ang = get_imu_rad(inertial);
                double lin = (2 * PI * drivetrain.wheel_radius) *
                (average(drivetrain.left_mg.get_position_all()) + average(drivetrain.right_mg.get_position_all()))/(2 * ticks_per_rotation);
                double dt = now() - last_time;

                if (dt > 0) {
                    compute(lin - last_lin, ang - last_ang, dt);

                    last_ang = ang;
                    last_lin = lin;
                    last_time = now();
                }

                pros::lcd::print(0, "(%f, %f), %f", this->get_pose().pos.x, this->get_pose().pos.y, this->get_pose().dir);


                delay_for(poll_delay);
            }
        }
    };

}
