#pragma once

#include <vector>

#include "math_utils.hpp"
#include "occupancy_grid.hpp"
#include "visibility_graph.hpp"

namespace sim {

class GNNPlanner {
public:
    GNNPlanner(double A = 10.0, double B = 1.0, double D = 1.0,
               double gamma = 2.0, double alpha = 2.0, double beta = 2.0,
               double V = 100.0, double E = 50.0, double dt = 0.01,
               int max_steps = 2000, double tol = 1e-4);

    [[nodiscard]] std::vector<Vec3> plan(const Vec3& start, const Vec3& goal,
                                         const OccupancyGrid& grid,
                                         const ObstacleField& obstacle_field,
                                         double visible_range = 20.0);
    [[nodiscard]] const std::vector<double>& activity() const { return activity_; }

private:
    [[nodiscard]] std::vector<std::vector<double>> compute_weight_matrix(const VisibilityGraph& graph) const;
    [[nodiscard]] std::vector<double> compute_excitation(const VisibilityGraph& graph,
                                                         const Vec3& start, const Vec3& goal) const;
    [[nodiscard]] std::vector<double> integrate_gnn(const std::vector<std::vector<double>>& weights,
                                                    const std::vector<double>& input);
    [[nodiscard]] std::vector<Vec3> extract_path_from_activities(const std::vector<double>& x,
                                                                 const VisibilityGraph& graph,
                                                                 const Vec3& goal) const;
    [[nodiscard]] bool can_reach_goal(int start_idx, int goal_idx,
                                      const std::vector<std::vector<int>>& adjacency,
                                      const std::vector<int>& visited) const;
    [[nodiscard]] std::vector<int> shortest_suffix(int start_idx, int goal_idx,
                                                   const std::vector<std::vector<int>>& adjacency,
                                                   const std::vector<int>& visited) const;

    double A_, B_, D_, gamma_, alpha_, beta_, V_, E_, dt_;
    int max_steps_;
    double tol_;
    std::vector<double> activity_;
};

}  // namespace sim
