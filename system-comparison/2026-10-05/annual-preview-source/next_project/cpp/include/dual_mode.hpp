#pragma once

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "math_utils.hpp"
#include "obstacles.hpp"

namespace sim {

class DualModeScheduler {
public:
    DualModeScheduler(double sensor_danger_threshold = 2.0,
                      double sensor_safe_threshold = 4.0,
                      double sdf_danger_threshold = 0.5,
                      double hysteresis_margin = 0.5)
        : sensor_danger_(sensor_danger_threshold),
          sensor_safe_(sensor_safe_threshold),
          sdf_danger_(sdf_danger_threshold),
          hysteresis_(hysteresis_margin) {}

    [[nodiscard]] std::string classify(double t, const std::array<double, 6>* sensor_reading,
                                       const ObstacleField* obstacle_field,
                                       const Vec3& position) {
        std::string previous = mode_;
        double sensor_min = std::numeric_limits<double>::infinity();
        if (sensor_reading) {
            for (double v : *sensor_reading) sensor_min = std::min(sensor_min, v);
        }

        double sdf_val = obstacle_field ? obstacle_field->signed_distance(position)
                                        : std::numeric_limits<double>::infinity();

        if (sensor_min < sensor_danger_ || sdf_val < sdf_danger_) {
            mode_ = "danger";
        } else if (previous == "danger") {
            if (sensor_min > sensor_safe_ + hysteresis_ &&
                sdf_val > sdf_danger_ + hysteresis_) {
                mode_ = "safe";
            }
        }

        history_.push_back({t, mode_});
        return mode_;
    }

    [[nodiscard]] bool is_danger() const { return mode_ == "danger"; }
    [[nodiscard]] bool is_safe() const { return mode_ == "safe"; }
    [[nodiscard]] const std::string& mode() const { return mode_; }

private:
    double sensor_danger_;
    double sensor_safe_;
    double sdf_danger_;
    double hysteresis_;
    std::string mode_ = "safe";
    std::vector<std::pair<double, std::string>> history_;
};

}  // namespace sim
