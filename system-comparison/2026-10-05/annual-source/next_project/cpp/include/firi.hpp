#pragma once

#include <vector>

#include "math_utils.hpp"
#include "obstacles.hpp"

namespace sim {

struct FIRICorridor {
    std::vector<Vec3> normals;
    std::vector<double> offsets;
    Vec3 start;
    Vec3 goal;
    double min_clearance = 0.0;
    Vec3 ellipsoid_center;
    double inscribed_radius = 0.0;

    [[nodiscard]] bool contains(const Vec3& point, double tol = 1e-8) const;
    [[nodiscard]] Vec3 project(Vec3 point, int max_iter = 8) const;
};

class FIRIRefiner {
public:
    FIRIRefiner(const ObstacleField& obstacle_field, double min_clearance,
                double sample_step = 0.2, int max_projection_iter = 10);

    [[nodiscard]] std::vector<Vec3> refine(const std::vector<Vec3>& path,
                                           const std::vector<Vec3>& seeds = {}) const;

private:
    [[nodiscard]] std::vector<Vec3> prepare_seeds(const std::vector<Vec3>& seeds) const;
    [[nodiscard]] FIRICorridor build_segment_corridor(const Vec3& start, const Vec3& goal) const;
    [[nodiscard]] std::vector<Vec3> refine_segment(const FIRICorridor& corridor) const;
    void compute_inscribed_ball(FIRICorridor& corridor) const;

    [[nodiscard]] std::vector<Vec3> sample_segment(const Vec3& start, const Vec3& goal, double step) const;
    [[nodiscard]] Vec3 closest_point_on_segment(const Vec3& a, const Vec3& b, const Vec3& p) const;
    [[nodiscard]] Vec3 push_out_if_needed(Vec3 point) const;
    [[nodiscard]] std::vector<Vec3> push_path_out_if_needed(const std::vector<Vec3>& path) const;
    [[nodiscard]] std::vector<Vec3> remove_duplicate_points(const std::vector<Vec3>& path) const;

    [[nodiscard]] bool analytic_separating_plane(const ObstacleVariant& obs, const Vec3& start,
                                                 const Vec3& goal, const std::vector<Vec3>& seed_points,
                                                 Vec3& normal, double& offset) const;
    [[nodiscard]] Vec3 nearest_obstacle_point(const ObstacleVariant& obs, const Vec3& start,
                                              const Vec3& goal) const;
    [[nodiscard]] Vec3 nearest_sphere_point(const Sphere& obs, const Vec3& start, const Vec3& goal) const;
    [[nodiscard]] Vec3 nearest_cylinder_point(const Cylinder& obs, const Vec3& start, const Vec3& goal) const;
    [[nodiscard]] Vec3 nearest_aabb_point(const AABB& obs, const Vec3& start, const Vec3& goal) const;
    [[nodiscard]] Vec3 safe_segment_normal(const Vec3& start, const Vec3& goal) const;

    const ObstacleField* obstacle_field_;
    double min_clearance_;
    double sample_step_;
    int max_projection_iter_;
};

}  // namespace sim
