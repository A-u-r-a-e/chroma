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
        pros::Distance sensor;
        double sensor_dist; // inches!

        const int sample_size;
        std::list<Measurement> cache;

        // get raw distance measurement in inches
        // returns -1 if nothing is detected
        double get_raw() {
            double mm_dist = sensor.get();
            if (mm_dist == 9999) return -1.0;
            double inch_dist = mm_dist / INCH_PER_MM;
            return inch_dist;
        }
    public:
        static const ms UPDATE_PERIOD; // minimum time to wait before collecting new data is effective

        // create an accuracy-based distance sensor object
        // port is for the hardware
        // sensor_dist is offset on the axis
        // sample size is amount of samples to use when averaging values
        TOF(uint8_t port, double sensor_dist, int sample_size = 3):
            sensor{port}, sensor_dist{sensor_dist}, sample_size{sample_size}
        {}

        // tries to update the cache once
        void update() {
            if (!cache.empty() && now() - cache.back().timestamp < UPDATE_PERIOD) return;
            double reading = get_raw();

            double distance = sensor_dist + reading;
            ms timestamp = now();

            cache.push_back(Measurement{.value=(reading > 0 ? distance : -1)});
            if (cache.size() > sample_size) cache.pop_front();
        }

        // get the current distance
        double get_top() {
            if (cache.empty()) return -1;
            double distance = cache.back().value;
            return distance;
        }

        // get most recent denoised data
        double get_denoised() {
            double amount = 0;
            double count = cache.size();
            for (auto& measurement: cache) {
                double m = measurement.value;
                amount += m > 0 ? m : 0;
            }
            amount /= count;
            return amount > 0 ? amount : -1;
        }

        // gets the sample variance of the recent data, -1 if no data
        double get_sample_variance() {
            double mean = get_denoised();
            if (mean < 0) return -1;
            double sum = 0;
            double count = cache.size();
            for (auto& measurement: cache) {
                double m = measurement.value;
                sum += m > 0 ? pow(m - mean, 2) : 0;
                if (m < 0) count--;
            }
            return count > 0 ? sum / count : -1;
        }
    };

}
