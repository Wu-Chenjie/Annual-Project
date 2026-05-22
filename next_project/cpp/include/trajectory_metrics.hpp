#pragma once

#include <vector>

#include "math_utils.hpp"

namespace sim {

struct TrajectoryMetrics {
    bool available = false;
    double path_length = 0.0;
    double mean_curvature = 0.0;
    double max_curvature = 0.0;
    double curvature_squared_integral = 0.0;
    double mean_jerk = 0.0;
    double max_jerk = 0.0;
    double jerk_squared_integral = 0.0;
    double snap_squared_integral = 0.0;
};

TrajectoryMetrics compute_trajectory_metrics(const std::vector<Vec3>& path, double sample_dt);

}  // namespace sim
