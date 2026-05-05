#include "artificial_potential_field.hpp"

namespace sim {

ImprovedArtificialPotentialField::ImprovedArtificialPotentialField(
    double k_rep_, double r_rep_, int n_decay_,
    double k_inter_, double s_inter_, double mu_escape_, double max_acc_)
    : k_rep(k_rep_), r_rep(r_rep_), n_decay(n_decay_),
      k_inter(k_inter_), s_inter(s_inter_), mu_escape(mu_escape_), max_acc(max_acc_) {}

void ImprovedArtificialPotentialField::reset() {
    in_escape_ = false;
    escape_counter_ = 0;
    escape_direction_ = Vec3{};
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

    return total_rep + f_escape;
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
    double decay = std::pow(goal_factor, n_decay);

    double term = std::max(1.0 / rho - 1.0 / r_rep, 0.0);
    double f_r1 = k_rep * term * (1.0 / (rho * rho)) * decay;

    double f_r2 = 0.0;
    if (n_decay >= 1 && goal_dist < r_rep) {
        f_r2 = 0.5 * n_decay * k_rep * (term * term)
               * std::pow(goal_factor, std::max(n_decay - 1, 0));
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
