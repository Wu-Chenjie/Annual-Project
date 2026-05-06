#pragma once

#include <utility>
#include <vector>

#include "math_utils.hpp"
#include "obstacles.hpp"

namespace sim {

class FormationAPF {
public:
    FormationAPF(double k_rep = 0.8, double r_rep = 3.0,
                 double alpha = 0.4, double beta = 0.6)
        : k_rep_(k_rep), r_rep_(r_rep), alpha_(alpha), beta_(beta) {}

    std::pair<Vec3, std::vector<Vec3>> compute_formation_avoidance(
        const Vec3& leader_pos,
        const std::vector<Vec3>& follower_positions,
        const Vec3& goal,
        const ObstacleField& obstacles,
        const std::vector<Vec3>& desired_offsets) const;

private:
    Vec3 centroid_repulsion(const Vec3& centroid, const Vec3& goal,
                            const ObstacleField& obstacles) const;
    std::vector<Vec3> relative_formation_forces(
        const std::vector<Vec3>& all_positions,
        const std::vector<Vec3>& desired_offsets) const;

    double k_rep_ = 0.8;
    double r_rep_ = 3.0;
    double alpha_ = 0.4;
    double beta_ = 0.6;
};

}  // namespace sim
