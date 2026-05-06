#include "artificial_potential_field.hpp"

namespace sim {

ImprovedArtificialPotentialField::ImprovedArtificialPotentialField(
    double k_rep_, double r_rep_, int n_decay_,
    double k_inter_, double s_inter_, double mu_escape_, double max_acc_,
    double k_comm_, double comm_range_, bool adaptive_n_decay_)
    : k_rep(k_rep_), r_rep(r_rep_), k_inter(k_inter_), s_inter(s_inter_),
      mu_escape(mu_escape_), max_acc(max_acc_), k_comm(k_comm_),
      comm_range(std::max(comm_range_, 1e-6)), n_decay(n_decay_),
      adaptive_n_decay(adaptive_n_decay_) {}

void ImprovedArtificialPotentialField::reset() {
    in_escape_ = false;
    escape_counter_ = 0;
    escape_direction_ = Vec3{};
    density_cache_.clear();
}

Vec3 ImprovedArtificialPotentialField::compute_avoidance_acceleration(
    const Vec3& position, const Vec3& goal, const ObstacleField& obstacles,
    const std::vector<Vec3>& other_positions) {

    Vec3 f_rep = obstacle_repulsion(position, goal, obstacles);
    Vec3 f_inter{};
    if (!other_positions.empty()) {
        f_inter = inter_drone_repulsion(position, other_positions);
    }

    Vec3 total_rep = f_rep + f_inter;
    double rep_norm = norm(total_rep);

    Vec3 f_escape{};
    if (rep_norm > 1e-6) {
        Vec3 to_goal = goal - position;
        double goal_dist = norm(to_goal);
        if (goal_dist > 1e-6) {
            Vec3 rep_dir = total_rep / rep_norm;
            Vec3 att_dir = to_goal / goal_dist;
            double cos_angle = dot(rep_dir, att_dir);
            if (cos_angle < std::cos(kEscapeAngleThreshold) && rep_norm > 1.0) {
                enter_escape(total_rep);
            } else {
                in_escape_ = false;
                escape_counter_ = 0;
            }
        }
    }

    if (in_escape_ && escape_counter_ < kEscapeHistory) {
        f_escape = compute_escape_force(total_rep);
        ++escape_counter_;
    } else {
        in_escape_ = false;
        escape_counter_ = 0;
    }

    Vec3 f_comm{};
    if (!other_positions.empty() && k_comm > 0.0) {
        f_comm = communication_constraint_force(position, other_positions);
    }

    Vec3 result = total_rep + f_escape + f_comm;
    double result_norm = norm(result);
    if (result_norm > max_acc) {
        result = result / result_norm * max_acc;
    }
    return result;
}

Vec3 ImprovedArtificialPotentialField::obstacle_repulsion(
    const Vec3& pos, const Vec3& goal, const ObstacleField& obs) const {

    double sd = obs.signed_distance(pos);
    if (sd >= r_rep) return Vec3{};

    double eps = 0.05;
    Vec3 grad{
        obs.signed_distance(Vec3{pos.x + eps, pos.y, pos.z})
        - obs.signed_distance(Vec3{pos.x - eps, pos.y, pos.z}),
        obs.signed_distance(Vec3{pos.x, pos.y + eps, pos.z})
        - obs.signed_distance(Vec3{pos.x, pos.y - eps, pos.z}),
        obs.signed_distance(Vec3{pos.x, pos.y, pos.z + eps})
        - obs.signed_distance(Vec3{pos.x, pos.y, pos.z - eps}),
    };
    grad = grad / (2.0 * eps);
    double grad_norm = norm(grad);
    if (grad_norm < 1e-10) return Vec3{};
    Vec3 rep_dir = grad / grad_norm;

    double rho = std::max(sd, 0.05);
    double goal_dist = norm(goal - pos);
    double goal_factor = std::min(goal_dist / r_rep, 1.0);
    double decay_n = adaptive_n_decay ? adaptive_decay_exponent(pos, obs) : static_cast<double>(n_decay);
    double decay = std::pow(goal_factor, decay_n);

    double term = std::max(1.0 / rho - 1.0 / r_rep, 0.0);
    double f_r1 = k_rep * term * (1.0 / (rho * rho)) * decay;

    double f_r2 = 0.0;
    if (decay_n >= 1.0 && goal_dist < r_rep) {
        f_r2 = 0.5 * decay_n * k_rep * (term * term)
               * std::pow(goal_factor, std::max(decay_n - 1.0, 0.0));
    }

    double f_mag = std::min(f_r1 + f_r2, max_acc);
    return rep_dir * f_mag;
}

Vec3 ImprovedArtificialPotentialField::inter_drone_repulsion(
    const Vec3& pos, const std::vector<Vec3>& others) const {

    Vec3 total{};
    for (const auto& other : others) {
        Vec3 diff = pos - other;
        double p = norm(diff);
        if (p < 1e-6 || p >= s_inter) continue;
        Vec3 dir = diff / p;
        double mag = k_inter * (1.0 / p - 1.0 / s_inter) * (1.0 / (p * p));
        total += dir * mag;
    }
    double total_norm = norm(total);
    if (total_norm > max_acc * 0.5) {
        total = total / total_norm * (max_acc * 0.5);
    }
    return total;
}

Vec3 ImprovedArtificialPotentialField::communication_constraint_force(
    const Vec3& pos, const std::vector<Vec3>& others) const {

    Vec3 total{};
    for (const auto& other : others) {
        Vec3 diff = other - pos;
        double dist = norm(diff);
        if (dist < 1e-6) continue;
        double gate = 0.5 * (1.0 + std::tanh((dist / comm_range - 0.8) / 0.05));
        double ratio = std::min(1.0, dist / comm_range);
        total += normalized(diff, Vec3{0.0, 0.0, 0.0}) * (k_comm * gate * ratio);
    }
    double limit = k_comm * static_cast<double>(others.size());
    double total_norm = norm(total);
    if (total_norm > limit && limit > 0.0) {
        total = total / total_norm * limit;
    }
    return total;
}

double ImprovedArtificialPotentialField::adaptive_decay_exponent(
    const Vec3& pos, const ObstacleField& obs) const {

    constexpr int agent_id = 0;
    auto it = density_cache_.find(agent_id);
    if (it != density_cache_.end()) {
        const DensityCacheEntry& cache = it->second;
        if (norm(pos - cache.last_pos) < r_rep / 4.0 && cache.step < 10) {
            density_cache_[agent_id] = DensityCacheEntry{cache.last_pos, cache.last_density, cache.step + 1};
            return 1.0 + 3.0 * std::min(cache.last_density, 1.0);
        }
    }

    double local_density = estimate_local_obstacle_density(pos, obs);
    density_cache_[agent_id] = DensityCacheEntry{pos, local_density, 0};
    return 1.0 + 3.0 * std::min(local_density, 1.0);
}

double ImprovedArtificialPotentialField::estimate_local_obstacle_density(
    const Vec3& pos, const ObstacleField& obs) const {

    double r = r_rep;
    constexpr int samples_per_axis = 3;
    int total = 0;
    int blocked = 0;
    double step = samples_per_axis > 1 ? (2.0 * r / static_cast<double>(samples_per_axis - 1)) : 1.0;

    for (int i = 0; i < samples_per_axis; ++i) {
        for (int j = 0; j < samples_per_axis; ++j) {
            for (int k = 0; k < samples_per_axis; ++k) {
                Vec3 offset{
                    -r + static_cast<double>(i) * step,
                    -r + static_cast<double>(j) * step,
                    -r + static_cast<double>(k) * step,
                };
                ++total;
                if (obs.signed_distance(pos + offset) < r) {
                    ++blocked;
                }
            }
        }
    }
    return total > 0 ? static_cast<double>(blocked) / static_cast<double>(total) : 0.0;
}

void ImprovedArtificialPotentialField::enter_escape(const Vec3& total_rep) {
    if (in_escape_) return;
    in_escape_ = true;
    escape_counter_ = 0;

    double rep_norm = norm(total_rep);
    if (rep_norm < 1e-6) {
        escape_direction_ = Vec3{1, 0, 0};
        return;
    }
    Vec3 rep_dir = total_rep / rep_norm;
    Vec3 basis = (std::abs(rep_dir.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    Vec3 perp = basis - rep_dir * dot(basis, rep_dir);
    double perp_norm = norm(perp);
    if (perp_norm < 1e-9) {
        perp = cross(rep_dir, Vec3{0, 0, 1});
        perp_norm = norm(perp);
    }
    escape_direction_ = (perp_norm > 1e-9) ? (perp / perp_norm) : Vec3{1, 0, 0};
}

Vec3 ImprovedArtificialPotentialField::compute_escape_force(const Vec3& total_rep) {
    double rep_norm = norm(total_rep);
    if (rep_norm < 1e-6) return Vec3{};
    Vec3 rep_dir = total_rep / rep_norm;
    double angle = 0.3 * escape_counter_;
    double ca = std::cos(angle), sa = std::sin(angle);
    Vec3 rotated = escape_direction_ * ca + cross(rep_dir, escape_direction_) * sa;
    Vec3 f_perp = rotated * rep_norm;
    return (total_rep + f_perp) * mu_escape;
}

}  // namespace sim
