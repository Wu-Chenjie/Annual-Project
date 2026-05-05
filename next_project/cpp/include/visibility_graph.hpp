#pragma once

#include <vector>

#include "math_utils.hpp"
#include "obstacles.hpp"

namespace sim {

class VisibilityGraph {
public:
    VisibilityGraph(int angular_res = 8, double buffer_zone = 0.3, int n_sphere = 12);

    void build(const Vec3& start, const Vec3& goal, const ObstacleField& obstacle_field,
               double visible_range = 20.0);
    void build_obstacle_graph(const ObstacleField& obstacle_field, double visible_range = 20.0);
    void set_start_goal(const Vec3& start, const Vec3& goal, const ObstacleField& obstacle_field,
                        double visible_range = 20.0);

    [[nodiscard]] bool is_visible(const Vec3& p1, const Vec3& p2,
                                  const ObstacleField& obstacle_field,
                                  double visible_range = 20.0) const;
    [[nodiscard]] std::size_t num_vertices() const { return vertices.size(); }

    std::vector<Vec3> vertices;
    std::vector<std::vector<int>> adjacency;
    std::vector<int> vertex_to_obs;

private:
    [[nodiscard]] std::vector<Vec3> extract_obstacle_vertices(const ObstacleVariant& obs) const;
    [[nodiscard]] std::vector<Vec3> aabb_vertices(const AABB& obs) const;
    [[nodiscard]] std::vector<Vec3> cylinder_vertices(const Cylinder& obs) const;
    [[nodiscard]] std::vector<Vec3> sphere_vertices(const Sphere& obs) const;
    [[nodiscard]] std::vector<Vec3> fibonacci_sphere(int n) const;

    int angular_res_;
    double buffer_zone_;
    int n_sphere_;
};

}  // namespace sim
