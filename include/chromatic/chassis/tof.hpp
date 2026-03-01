#pragma once
#include "api.h"
#include "chromatic/core.hpp"
#include "chromatic/shorthands.hpp"

namespace chromatic {

    // Tutorial:
    // In the asynchronous loop, run update_data() every iteration
    // When you need the data, call get_normalized().
    struct TOF {
    public:
        struct Measurement {
            double value;
            ms timestamp = now();
        };
    private:
        const ms UPDATE_PERIOD; // minimum time to wait before collecting new data is effective
        pros::Distance sensor;
        double sensor_dist; // inches!

        const int sample_size;
        std::list<Measurement> cache;

        // get raw distance measurement in inches
        double get_raw() {
            double mm_dist = sensor.get();
            double inch_dist = mm_dist / INCH_PER_MM;
            return inch_dist;
        }
    public:

        // create an accuracy-based distance sensor object
        // port is for the hardware
        // sensor_dist is offset on the axis
        // sample size is amount of samples to use when averaging values
        TOF(uint8_t port, double sensor_dist, int sample_size = 3, const ms update_period = 35) : sensor{port}, sensor_dist{sensor_dist}, sample_size{sample_size}, UPDATE_PERIOD{update_period} {}

        // tries to update the cache once
        void update() {

            if (!cache.empty() && now() - cache.back().timestamp < UPDATE_PERIOD) return;

            double distance = sensor_dist + get_raw();
            ms timestamp = now();

            cache.push_back(Measurement{.value=distance});
            if (cache.size() > sample_size)cache.pop_front();
        }

        // get the current distance
        double get_top() {
            double distance = cache.back().value;
            return distance;
        }

        // get most recent denoised data
        double get_denoised() {
            double amount = 0;
            for (auto& measurement: cache) {
                amount += measurement.value;
            }

            amount /= sample_size;
            return amount;
        }

        // can block, force to get some non moving data
        // dur is the amount of ms since the bottom of the queue was added
        // max_delta is the max amount of change in the measurement
        [[deprecated("deadloop issues, use denoise and updates instead")]]
        double poll(ms max_dt = 150, double max_delta = 1, ms timeout = 200) {
            ms real_dt = std::max(max_dt, UPDATE_PERIOD * sample_size);

            auto delta = [&] {
                double min = 1e9, max = -1;
                for (auto& measurement: cache) {
                    double val = measurement.value;
                    if (val < min) min = val;
                    if (val > max) max = val;
                }
                return max - min;
            };

            ms pre = now();
            while (now() - cache.front().timestamp > real_dt || delta() > max_delta) {
                if (now() - pre >= timeout) break;
                update();
                delay_for(UPDATE_PERIOD);
            }

            return get_denoised();
        }

    };



}
