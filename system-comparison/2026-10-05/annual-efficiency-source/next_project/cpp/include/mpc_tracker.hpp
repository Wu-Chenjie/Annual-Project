#pragma once

#include <string>
#include <vector>

#include "math_utils.hpp"

namespace sim {

struct MPCFeasibilityResult {
    bool evaluated = false;
    bool feasible = false;
    double tracking_rms_proxy = 0.0;
    double max_velocity_violation = 0.0;
    double max_acceleration_violation = 0.0;
    double saturation_ratio = 0.0;
    std::string recommendation = "missing_trajectory";
};

MPCFeasibilityResult evaluate_mpc_feasibility(const std::vector<Vec3>& path,
                                              double sample_dt,
                                              double max_speed,
                                              double max_acceleration,
                                              double rms_limit);

}  // namespace sim
