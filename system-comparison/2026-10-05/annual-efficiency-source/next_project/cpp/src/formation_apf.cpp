#include "formation_apf.hpp"

#include <algorithm>

namespace sim {

std::pair<Vec3, std::vector<Vec3>> FormationAPF::compute_formation_avoidance(
    const Vec3& leader_pos,
    const std::vector<Vec3>& follower_positions,
    const Vec3& goal,
    const ObstacleField& obstacles,
    const std::vector<Vec3>& desired_offsets) const {

    std::vector<Vec3> all_positions;
    all_positions.reserve(1 + follower_positions.size());
    all_positions.push_back(leader_pos);
    all_positions.insert(all_positions.end(), follower_positions.begin(), follower_positions.end());

    Vec3 centroid{};
    for (const auto& pos : all_positions) centroid += pos;
    centroid /= static_cast<double>(std::max<std::size_t>(1, all_positions.size()));

    Vec3 centroid_force = centroid_repulsion(centroid, goal, obstacles);
    std::vector<Vec3> relative_forces = relative_formation_forces(all_positions, desired_offsets);

    Vec3 leader_force = centroid_force * alpha_;
    if (!relative_forces.empty()) leader_force += relative_forces[0] * beta_;

    std::vector<Vec3> follower_forces;
    follower_forces.reserve(follower_positions.size());
    for (std::size_t i = 0; i < follower_positions.size(); ++i) {
        Vec3 total = centroid_force * alpha_;
        if (i + 1 < relative_forces.size()) total += relative_forces[i + 1] * beta_;
        follower_forces.push_back(total);
    }
    return {leader_force, follower_forces};
}

Vec3 FormationAPF::centroid_repulsion(
    const Vec3& centroid, const Vec3& goal, const ObstacleField& obstacles) const {

    double sd = obstacles.signed_distance(centroid);
    if (sd >= r_rep_) return Vec3{};

    constexpr double eps = 0.05;
    Vec3 grad{
        obstacles.signed_distance(centroid + Vec3{eps, 0.0, 0.0})
        - obstacles.signed_distance(centroid - Vec3{eps, 0.0, 0.0}),
        obstacles.signed_distance(centroid + Vec3{0.0, eps, 0.0})
        - obstacles.signed_distance(centroid - Vec3{0.0, eps, 0.0}),
        obstacles.signed_distance(centroid + Vec3{0.0, 0.0, eps})
        - obstacles.signed_distance(centroid - Vec3{0.0, 0.0, eps}),
    };
    grad /= (2.0 * eps);
    double grad_norm = norm(grad);
    if (grad_norm < 1e-10) return Vec3{};
    Vec3 rep_dir = grad / grad_norm;

    double rho = std::max(sd, 0.05);
    double goal_dist = norm(goal - centroid);
    double goal_factor = std::min(goal_dist / r_rep_, 1.0);
    double term = std::max(1.0 / rho - 1.0 / r_rep_, 0.0);
    double f_r1_mag = k_rep_ * term * (1.0 / (rho * rho)) * std::pow(goal_factor, 2.0);
    Vec3 f_r1_vec = rep_dir * f_r1_mag;

    Vec3 f_r2_vec{};
    if (goal_dist < r_rep_) {
        double f_r2_mag = k_rep_ * (term * term) * goal_factor;
        f_r2_vec = normalized(goal - centroid, Vec3{}) * f_r2_mag;
    }
    return f_r1_vec + f_r2_vec;
}

std::vector<Vec3> FormationAPF::relative_formation_forces(
    const std::vector<Vec3>& all_positions,
    const std::vector<Vec3>& desired_offsets) const {

    std::vector<Vec3> forces(all_positions.size(), Vec3{});
    if (all_positions.empty()) return forces;

    Vec3 leader_pos = all_positions[0];
    for (std::size_t i = 0; i < all_positions.size(); ++i) {
        Vec3 desired = (i < desired_offsets.size()) ? (leader_pos + desired_offsets[i]) : all_positions[i];
        forces[i] = desired - all_positions[i];
    }
    return forces;
}

}  // namespace sim
