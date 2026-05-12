#pragma once

#include <iostream>
#include <cmath>
#include <numeric>
#include <vector>

namespace algorithms {

    class PlanarImuIntegrator {
    public:
        PlanarImuIntegrator() : theta_(0.0f), gyro_offset_(0.0f) {}

        // Integrates gyro_z over time, factoring in the calculated bias
        void update(float gyro_z, double dt) {
            float adjusted_gyro_z = gyro_z - gyro_offset_;
            theta_ += adjusted_gyro_z * dt;
        }

        // Calibrates the gyroscope by computing the average from static samples
        void setCalibration(const std::vector<float>& gyro) {
            if (gyro.empty()) {
                gyro_offset_ = 0.0f;
                return;
            }
            float sum = std::accumulate(gyro.begin(), gyro.end(), 0.0f);
            gyro_offset_ = sum / gyro.size();
        }

        // Returns the current estimated yaw
        [[nodiscard]] float getYaw() const {
            return theta_;
        }

        // Resets orientation and calibration
        void reset() {
            theta_ = 0.0f;
            gyro_offset_ = 0.0f;
        }

    private:
        float theta_;       // Integrated yaw angle (radians)
        float gyro_offset_; // Estimated gyro bias
    };

} // namespace algorithms