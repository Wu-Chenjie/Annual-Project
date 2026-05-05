#include "gnn_planner.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

#include "astar_planner.hpp"

namespace sim {

GNNPlanner::GNNPlanner(double A, double B, double D, double gamma, double alpha, double beta,
                       double V, double E, double dt, int max_steps, double tol)
    : A_(A), B_(B), D_(D), gamma_(gamma), alpha_(alpha), beta_(beta),
      V_(V), E_(E), dt_(dt), max_steps_(max_steps), tol_(tol) {}

std::vector<Vec3> GNNPlanner::plan(const Vec3& start, const Vec3& goal,
                                   const OccupancyGrid& grid,
                                   const ObstacleField& obstacle_field,
                                   double visible_range) {
    (void)grid;
    VisibilityGraph graph;
    graph.build(start, goal, obstacle_field, visible_range);
    if (graph.num_vertices() < 2) return {};

    auto weights = compute_weight_matrix(graph);
    auto input = compute_excitation(graph, start, goal);
    auto x = integrate_gnn(weights, input);
    auto path = extract_path_from_activities(x, graph, goal);
    return smooth_path(path, 4);
}

std::vector<std::vector<double>> GNNPlanner::compute_weight_matrix(const VisibilityGraph& graph) const {
    std::size_t n = graph.num_vertices();
    std::vector<std::vector<double>> weights(n, std::vector<double>(n, 0.0));
    for (std::size_t i = 0; i < n; ++i) {
        for (int j_raw : graph.adjacency[i]) {
            std::size_t j = static_cast<std::size_t>(j_raw);
            if (i >= j || j >= n) continue;
            double d = norm(graph.vertices[i] - graph.vertices[j]);
            double w = 1.0 / std::pow(1.0 + d, gamma_);
            weights[i][j] = w;
            weights[j][i] = w;
        }
    }
    return weights;
}

std::vector<double> GNNPlanner::compute_excitation(const VisibilityGraph& graph,
                                                   const Vec3& start, const Vec3& goal) const {
    std::size_t n = graph.num_vertices();
    std::vector<double> input(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        double d_goal = norm(graph.vertices[i] - goal);
        double d_path = norm(graph.vertices[i] - start);
        input[i] = V_ / std::pow(1.0 + d_goal, alpha_) / std::pow(1.0 + d_path, beta_);
    }
    if (n > 1) input[1] = E_;
    return input;
}

std::vector<double> GNNPlanner::integrate_gnn(const std::vector<std::vector<double>>& weights,
                                              const std::vector<double>& input) {
    std::size_t n = input.size();
    std::vector<double> x(n, 0.0);
    for (int step = 0; step < max_steps_; ++step) {
        std::vector<double> next = x;
        double max_delta = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            double lateral = 0.0;
            for (std::size_t j = 0; j < n; ++j) lateral += weights[i][j] * x[j];
            double dx = -A_ * x[i] + (B_ - x[i]) * (input[i] + lateral) - (D_ + x[i]) * 0.0;
            next[i] = clamp(x[i] + dt_ * dx, 0.0, B_);
            max_delta = std::max(max_delta, std::abs(next[i] - x[i]));
        }
        x = std::move(next);
        if (max_delta < tol_) break;
    }
    activity_ = x;
    return x;
}

std::vector<Vec3> GNNPlanner::extract_path_from_activities(const std::vector<double>& x,
                                                           const VisibilityGraph& graph,
                                                           const Vec3& goal) const {
    if (graph.num_vertices() < 2) return {};
    int current = 0;
    int goal_idx = 1;
    std::vector<int> visited(static_cast<std::size_t>(graph.num_vertices()), 0);
    visited[0] = 1;
    std::vector<Vec3> path{graph.vertices[0]};

    int max_iter = static_cast<int>(graph.num_vertices() * 2);
    for (int iter = 0; iter < max_iter; ++iter) {
        if (current == goal_idx) break;
        int best = -1;
        double best_activity = -1.0;
        for (int nb : graph.adjacency[static_cast<std::size_t>(current)]) {
            if (nb < 0 || nb >= static_cast<int>(graph.num_vertices())) continue;
            if (visited[static_cast<std::size_t>(nb)]) continue;
            auto next_visited = visited;
            next_visited[static_cast<std::size_t>(current)] = 1;
            if (nb != goal_idx && !can_reach_goal(nb, goal_idx, graph.adjacency, next_visited)) continue;
            if (x[static_cast<std::size_t>(nb)] > best_activity) {
                best_activity = x[static_cast<std::size_t>(nb)];
                best = nb;
            }
        }
        if (best < 0) {
            auto suffix = shortest_suffix(current, goal_idx, graph.adjacency, visited);
            if (suffix.empty()) break;
            for (int idx : suffix) path.push_back(graph.vertices[static_cast<std::size_t>(idx)]);
            current = goal_idx;
            break;
        }
        visited[static_cast<std::size_t>(best)] = 1;
        current = best;
        path.push_back(graph.vertices[static_cast<std::size_t>(current)]);
    }

    if (current != goal_idx && (path.empty() || norm(path.back() - goal) > 1e-9)) {
        path.push_back(goal);
    }
    return path;
}

bool GNNPlanner::can_reach_goal(int start_idx, int goal_idx,
                                const std::vector<std::vector<int>>& adjacency,
                                const std::vector<int>& visited) const {
    if (start_idx == goal_idx) return true;
    std::queue<int> q;
    std::vector<int> seen(visited.size(), 0);
    q.push(start_idx);
    seen[static_cast<std::size_t>(start_idx)] = 1;
    while (!q.empty()) {
        int node = q.front();
        q.pop();
        for (int nb : adjacency[static_cast<std::size_t>(node)]) {
            if (nb == goal_idx) return true;
            std::size_t idx = static_cast<std::size_t>(nb);
            if (visited[idx] || seen[idx]) continue;
            seen[idx] = 1;
            q.push(nb);
        }
    }
    return false;
}

std::vector<int> GNNPlanner::shortest_suffix(int start_idx, int goal_idx,
                                             const std::vector<std::vector<int>>& adjacency,
                                             const std::vector<int>& visited) const {
    std::queue<int> q;
    std::vector<int> parent(visited.size(), -2);
    q.push(start_idx);
    parent[static_cast<std::size_t>(start_idx)] = -1;
    while (!q.empty()) {
        int node = q.front();
        q.pop();
        if (node == goal_idx) break;
        for (int nb : adjacency[static_cast<std::size_t>(node)]) {
            std::size_t idx = static_cast<std::size_t>(nb);
            if (visited[idx] && nb != start_idx) continue;
            if (parent[idx] != -2) continue;
            parent[idx] = node;
            q.push(nb);
        }
    }
    if (parent[static_cast<std::size_t>(goal_idx)] == -2) return {};

    std::vector<int> suffix;
    int node = goal_idx;
    while (node != start_idx) {
        suffix.push_back(node);
        node = parent[static_cast<std::size_t>(node)];
        if (node < 0) return {};
    }
    std::reverse(suffix.begin(), suffix.end());
    return suffix;
}

}  // namespace sim
