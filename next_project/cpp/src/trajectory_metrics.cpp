#include "trajectory_metrics.hpp"

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

double max_norm(const std::vector<Vec3>& values) {
    double out = 0.0;
    for (const auto& value : values) out = std::max(out, norm(value));
    return out;
}

double squared_integral(const std::vector<Vec3>& values, double dt) {
    double out = 0.0;
    for (std::size_t i = 1; i < values.size(); ++i) {
        out += dot(values[i], values[i]) * dt;
    }
    return out;
}

}  // namespace

TrajectoryMetrics compute_trajectory_metrics(const std::vector<Vec3>& path, double sample_dt) {
    TrajectoryMetrics out;
    if (path.empty()) return out;
    out.available = true;
    const double dt = std::max(sample_dt, 1e-6);

    for (std::size_t i = 1; i < path.size(); ++i) {
        out.path_length += norm(path[i] - path[i - 1]);
    }

    if (path.size() >= 3) {
        double curvature_sum = 0.0;
        int curvature_count = 0;
        for (std::size_t i = 1; i + 1 < path.size(); ++i) {
            const Vec3 a = path[i] - path[i - 1];
            const Vec3 b = path[i + 1] - path[i];
            const double la = norm(a);
            const double lb = norm(b);
            const double chord = norm(path[i + 1] - path[i - 1]);
            double curvature = 0.0;
            if (la > 1e-9 && lb > 1e-9 && chord > 1e-9) {
                curvature = 2.0 * norm(cross(a, b)) / std::max(la * lb * chord, 1e-9);
            }
            curvature_sum += curvature;
            out.max_curvature = std::max(out.max_curvature, curvature);
            out.curvature_squared_integral += curvature * curvature * dt;
            ++curvature_count;
        }
        out.mean_curvature = curvature_count > 0 ? curvature_sum / static_cast<double>(curvature_count) : 0.0;
    }

    const auto velocities = differentiate(path, dt);
    const auto accelerations = differentiate(velocities, dt);
    const auto jerks = differentiate(accelerations, dt);
    const auto snaps = differentiate(jerks, dt);
    double jerk_sum = 0.0;
    for (const auto& jerk : jerks) jerk_sum += norm(jerk);
    out.mean_jerk = !jerks.empty() ? jerk_sum / static_cast<double>(jerks.size()) : 0.0;
    out.max_jerk = max_norm(jerks);
    out.jerk_squared_integral = squared_integral(jerks, dt);
    out.snap_squared_integral = squared_integral(snaps, dt);
    return out;
}

}  // namespace sim
