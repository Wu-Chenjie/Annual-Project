#include "mpc_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace sim {

namespace {

std::vector<Vec3> differentiate(const std::vector<Vec3>& values, double dt) {
    std::vector<Vec3> out(values.size());
    if (values.size() < 2) return out;
    for (std::size_t i = 1; i < values.size(); ++i) {
        out[i] = (values[i] - values[i - 1]) / dt;
    }
    out[0] = out[1];
    return out;
}

}  // namespace

MPCFeasibilityResult evaluate_mpc_feasibility(const std::vector<Vec3>& path,
                                              double sample_dt,
                                              double max_speed,
                                              double max_acceleration,
                                              double rms_limit) {
    MPCFeasibilityResult out;
    if (path.size() < 2) return out;

    out.evaluated = true;
    const double dt = std::max(sample_dt, 1e-6);
    const double speed_limit = std::max(max_speed, 1e-6);
    const double acc_limit = std::max(max_acceleration, 1e-6);
    const auto velocities = differentiate(path, dt);
    const auto accelerations = differentiate(velocities, dt);

    int saturated_count = 0;
    double rms_sum = 0.0;
    for (std::size_t i = 0; i < path.size(); ++i) {
        const double speed = norm(velocities[i]);
        const double acc = norm(accelerations[i]);
        out.max_velocity_violation = std::max(out.max_velocity_violation, speed - speed_limit);
        out.max_acceleration_violation = std::max(out.max_acceleration_violation, acc - acc_limit);
        if (speed > speed_limit || acc > acc_limit) ++saturated_count;
        const double normalized_acc = std::min(acc / acc_limit, 2.0);
        rms_sum += normalized_acc * normalized_acc;
    }

    out.max_velocity_violation = std::max(out.max_velocity_violation, 0.0);
    out.max_acceleration_violation = std::max(out.max_acceleration_violation, 0.0);
    out.saturation_ratio = static_cast<double>(saturated_count) / static_cast<double>(path.size());
    out.tracking_rms_proxy = std::sqrt(rms_sum / static_cast<double>(path.size()));
    out.feasible = out.max_velocity_violation == 0.0
        && out.max_acceleration_violation == 0.0
        && out.tracking_rms_proxy <= std::max(rms_limit, 0.0);
    out.recommendation = out.feasible ? "continue_mpc_prototype" : "defer_online_mpc";
    return out;
}

}  // namespace sim
