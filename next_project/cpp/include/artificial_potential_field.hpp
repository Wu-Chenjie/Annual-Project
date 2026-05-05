#pragma once

#include <cmath>
#include <vector>

#include "math_utils.hpp"
#include "obstacles.hpp"

namespace sim {

class ImprovedArtificialPotentialField {
public:
    ImprovedArtificialPotentialField(double k_rep = 0.8, double r_rep = 2.5, int n_decay = 2,
                                     double k_inter = 0.2, double s_inter = 2.0,
                                     double mu_escape = 0.5, double max_acc = 8.0);

    void reset();

    Vec3 compute_avoidance_acceleration(const Vec3& position, const Vec3& goal,
                                        const ObstacleField& obstacles,
                                        const std::vector<Vec3>& other_positions = {});

    double k_rep, r_rep, k_inter, s_inter, mu_escape, max_acc;
    int n_decay;

private:
    Vec3 obstacle_repulsion(const Vec3& pos, const Vec3& goal, const ObstacleField& obs) const;
    Vec3 inter_drone_repulsion(const Vec3& pos, const std::vector<Vec3>& others) const;
    void enter_escape(const Vec3& total_rep);
    Vec3 compute_escape_force(const Vec3& total_rep);

    bool in_escape_ = false;
    int escape_counter_ = 0;
    Vec3 escape_direction_{};
    static constexpr double kEscapeAngleThreshold = 2.95;  // ~0.94π
    static constexpr int kEscapeHistory = 20;
};

}  // namespace sim
