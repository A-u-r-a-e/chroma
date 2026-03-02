#pragma  once
#include "chromatic/chassis/odometry.hpp"

namespace chromatic {

    struct EncodersIMU : public Odometry {
    private:
        Differential& drivebase;
        pros::IMU& inertial;
        const double imu_drift;

        double last_ang;
        double last_lin;
        ms last_time;

        std::atomic<bool> calibrated;
    public:

        EncodersIMU(
            Differential &drivebase, pros::IMU &inertial, double imu_drift = 1):
            drivebase(drivebase), inertial(inertial), imu_drift(imu_drift)
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
        void localize(ms pollrate) override {
            while (!calibrated);
            active = true;
            while (active && calibrated) {
                // these values are completely independent from posev
                double ang = get_imu_rad(inertial);
                double lin = (PI * drivebase.wheel_radius) * (average(drivebase.left_mg.get_position_all()) + average(drivebase.right_mg.get_position_all()))/(2 * drivebase.get_ticks_per_wheel_rev());

                Vec dpos{lin - last_lin, 0};
                double dang = (ang - last_ang);
                double dt = now() - last_time;

                if (dt > 0) {
                    integrate(dpos, imu_drift * dang, dt);
                    last_ang = ang;
                    last_lin = lin;
                    last_time = now();
                }
                pros::lcd::print(0, "(%f, %f), %f", this->get_pose().pos.x, this->get_pose().pos.y, to_deg(this->get_pose().dir));

                delay_for(pollrate);
            }
        }
    };

}
