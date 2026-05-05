#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

#include "esdf.hpp"
#include "math_utils.hpp"
#include "occupancy_grid.hpp"

namespace sim {

struct AStarResult {
    std::vector<Vec3> path;
    bool success = false;
};

inline AStarResult astar_plan(const Vec3& start, const Vec3& goal, const OccupancyGrid& grid) {
    auto [sx, sy, sz] = grid.world_to_index(start);
    auto [gx, gy, gz] = grid.world_to_index(goal);

    AStarResult result;
    if (grid.is_occupied(sx, sy, sz) || grid.is_occupied(gx, gy, gz)) return result;

    using Key = std::array<int, 3>;
    struct KeyHash {
        std::size_t operator()(const Key& k) const {
            return (static_cast<std::size_t>(k[0]) * 73856093)
                 ^ (static_cast<std::size_t>(k[1]) * 19349663)
                 ^ (static_cast<std::size_t>(k[2]) * 83492791);
        }
    };
    struct KeyEq {
        bool operator()(const Key& a, const Key& b) const {
            return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
        }
    };

    struct Node {
        double f, g;
        Key idx;
        bool operator>(const Node& o) const { return f > o.f; }
    };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::unordered_map<Key, double, KeyHash, KeyEq> g_score;
    std::unordered_map<Key, Key, KeyHash, KeyEq> parent;
    std::unordered_map<Key, bool, KeyHash, KeyEq> visited;

    Key start_key{sx, sy, sz}, goal_key{gx, gy, gz};
    auto heuristic = [&](const Key& a, const Key& b) {
        double dx = (a[0] - b[0]) * grid.resolution;
        double dy = (a[1] - b[1]) * grid.resolution;
        double dz = (a[2] - b[2]) * grid.resolution;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    double start_h = heuristic(start_key, goal_key);
    pq.push({start_h, 0.0, start_key});
    g_score[start_key] = 0.0;

    // Check for CostAwareGrid via virtual extra_cost method
    auto* cag = dynamic_cast<const CostAwareGrid*>(&grid);

    while (!pq.empty()) {
        auto [f, g, cur] = pq.top(); pq.pop();
        if (visited[cur]) continue;
        visited[cur] = true;

        if (cur == goal_key) {
            result.success = true;
            Key node = cur;
            while (parent.count(node)) {
                result.path.push_back(grid.index_to_world(node[0], node[1], node[2]));
                node = parent[node];
            }
            result.path.push_back(grid.index_to_world(start_key[0], start_key[1], start_key[2]));
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }

        for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) continue;
            int nx_i = cur[0] + dx, ny_i = cur[1] + dy, nz_i = cur[2] + dz;
            if (grid.is_occupied(nx_i, ny_i, nz_i)) continue;
            Key nb{nx_i, ny_i, nz_i};
            if (visited[nb]) continue;

            double cost = std::sqrt(dx*dx + dy*dy + dz*dz) * grid.resolution;
            if (cag) cost += cag->extra_cost(nx_i, ny_i, nz_i);

            double ng = g + cost;
            auto it = g_score.find(nb);
            if (it == g_score.end() || ng < it->second) {
                g_score[nb] = ng;
                parent[nb] = cur;
                pq.push({ng + heuristic(nb, goal_key), ng, nb});
            }
        }
    }

    return result;
}

inline std::vector<Vec3> smooth_path(const std::vector<Vec3>& path, int num_insert = 4) {
    if (path.size() < 3) return path;
    int K = static_cast<int>(path.size());
    std::vector<Vec3> out;
    out.push_back(path[0]);

    for (int i = 0; i < K - 1; ++i) {
        Vec3 p0 = (i >= 1) ? path[i - 1] : (path[0] * 2.0 - path[1]);
        Vec3 p1 = path[i];
        Vec3 p2 = path[i + 1];
        Vec3 p3 = (i + 2 < K) ? path[i + 2] : (path[K - 1] * 2.0 - path[K - 2]);

        for (int j = 0; j < num_insert; ++j) {
            double t = (j + 1) / static_cast<double>(num_insert + 1);
            double tt = t * t, ttt = tt * t;
            Vec3 pt = (p1 * 2.0 + (p2 - p0) * t
                       + (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * tt
                       + (p3 - p1 * 3.0 + p2 * 3.0 - p0) * ttt) * 0.5;
            out.push_back(pt);
        }
        out.push_back(p2);
    }
    return out;
}

}  // namespace sim
