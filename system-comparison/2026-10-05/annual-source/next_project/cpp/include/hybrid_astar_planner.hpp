#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <queue>
#include <unordered_map>
#include <vector>

#include "math_utils.hpp"
#include "occupancy_grid.hpp"

namespace sim {

inline constexpr double kPi = 3.141592653589793238462643383279502884;

struct HybridAStarResult {
    std::vector<Vec3> path;
    bool success = false;
};

class HybridAStarPlanner {
public:
    HybridAStarPlanner(double v_max = 2.0, double v_z_max = 1.0, double omega_max = 1.0472,
                       double dt_primitive = 0.5, int n_heading_bins = 72,
                       int max_iter = 10000, int analytic_interval = 50,
                       double reverse_penalty = 0.5, double dir_change_penalty = 0.2)
        : v_max_(v_max), v_z_max_(v_z_max), omega_max_(omega_max),
          dt_(dt_primitive), n_heading_bins_(n_heading_bins),
          max_iter_(max_iter), analytic_interval_(analytic_interval),
          reverse_penalty_(reverse_penalty), dir_change_penalty_(dir_change_penalty),
          heading_res_(2.0 * kPi / n_heading_bins),
          v_diag_(v_max / std::sqrt(2.0)) {
        build_primitives();
    }

    HybridAStarResult plan(const Vec3& start, const Vec3& goal, const OccupancyGrid& grid,
                           int seed = 42);

private:
    struct State { double x, y, z, psi; };
    using StateKey = std::array<int, 4>;
    struct KeyHash {
        std::size_t operator()(const StateKey& k) const {
            return (static_cast<std::size_t>(k[0]) * 73856093)
                 ^ (static_cast<std::size_t>(k[1]) * 19349663)
                 ^ (static_cast<std::size_t>(k[2]) * 83492791)
                 ^ (static_cast<std::size_t>(k[3]) * 52520179);
        }
    };
    struct KeyEq {
        bool operator()(const StateKey& a, const StateKey& b) const {
            return a[0] == b[0] && a[1] == b[1] && a[2] == b[2] && a[3] == b[3];
        }
    };

    struct Primitive { double vx_b, vy_b, vz, omega; };
    struct SearchNode { double f, g; StateKey key; bool operator>(const SearchNode& o) const { return f > o.f; } };
    using ParentMap = std::unordered_map<StateKey, std::pair<StateKey, int>, KeyHash, KeyEq>;
    using GScoreMap = std::unordered_map<StateKey, double, KeyHash, KeyEq>;
    using HoloMap = std::unordered_map<StateKey, double, KeyHash, KeyEq>;

    void build_primitives();
    State propagate(const State& s, int pi, std::vector<Vec3>& traj) const;
    bool collision_free_trajectory(const std::vector<Vec3>& traj, const OccupancyGrid& grid) const;
    StateKey state_key(const Vec3& pos, double psi, const OccupancyGrid& grid) const;
    double heuristic(const State& s, const Vec3& goal,
                     const std::unordered_map<StateKey, double, KeyHash, KeyEq>& holo) const;
    std::unordered_map<StateKey, double, KeyHash, KeyEq>
        precompute_holonomic_heuristic(const Vec3& goal, const OccupancyGrid& grid) const;
    State analytic_expand(const State& s, const Vec3& goal, const OccupancyGrid& grid) const;
    double primitive_cost(int pi, int prev_pi) const;
    std::vector<Vec3> extract_path(const StateKey& gk,
        const std::unordered_map<StateKey, std::pair<StateKey, int>, KeyHash, KeyEq>& parent,
        const State& start_state, const OccupancyGrid& grid) const;
    State reconstruct_state(const StateKey& key,
        const std::unordered_map<StateKey, std::pair<StateKey, int>, KeyHash, KeyEq>& parent,
        const State& start_state, const OccupancyGrid& grid) const;
    std::vector<Vec3> downsample_path(const std::vector<Vec3>& path, double spacing) const;

    double v_max_, v_z_max_, omega_max_, dt_;
    int n_heading_bins_, max_iter_, analytic_interval_;
    double reverse_penalty_, dir_change_penalty_;
    double heading_res_, v_diag_;
    std::vector<Primitive> primitives_;
    Vec3 goal_pos_{};
    double goal_psi_ = 0.0;
    const OccupancyGrid* grid_ = nullptr;
};

// ====================================================================
// Implementation
// ====================================================================

inline void HybridAStarPlanner::build_primitives() {
    double v = v_max_, vz = v_z_max_, w = omega_max_, vd = v_diag_;
    primitives_ = {
        {v, 0, 0, 0}, {v/2, 0, 0, 0}, {0, v, 0, 0}, {0, -v, 0, 0},
        {vd, vd, 0, 0}, {vd, -vd, 0, 0}, {-v/2, 0, 0, 0},
        {0, 0, vz, 0}, {0, 0, -vz, 0},
        {v/2, 0, 0, w}, {v/2, 0, 0, -w}, {0, 0, 0, w}, {0, 0, 0, -w},
    };
}

inline HybridAStarPlanner::State HybridAStarPlanner::propagate(
    const State& s, int pi, std::vector<Vec3>& traj) const {
    const auto& p = primitives_[pi];
    double cpsi = std::cos(s.psi), spsi = std::sin(s.psi);
    double dt = dt_;
    double nz = s.z + p.vz * dt;

    if (std::abs(p.omega) < 1e-9) {
        double nx = s.x + (p.vx_b * cpsi - p.vy_b * spsi) * dt;
        double ny = s.y + (p.vx_b * spsi + p.vy_b * cpsi) * dt;
        double npsi = s.psi;
        int ns = std::max(2, static_cast<int>(dt / 0.05));
        for (int i = 0; i <= ns; ++i) {
            double t = i * dt / ns;
            traj.push_back({s.x + (p.vx_b*cpsi - p.vy_b*spsi)*t,
                            s.y + (p.vx_b*spsi + p.vy_b*cpsi)*t,
                            s.z + p.vz*t});
        }
        return {nx, ny, nz, npsi};
    } else {
        double npsi = std::fmod(s.psi + p.omega * dt, 2.0 * kPi);
        if (npsi < 0) npsi += 2.0 * kPi;
        double sn = std::sin(npsi), cn = std::cos(npsi);
        double iw = 1.0 / p.omega;
        double nx = s.x + iw * (p.vx_b*(sn-spsi) + p.vy_b*(cn-cpsi));
        double ny = s.y + iw * (p.vx_b*(cpsi-cn) + p.vy_b*(sn-spsi));
        int ns = std::max(2, static_cast<int>(std::abs(p.omega) * dt / 0.05));
        for (int i = 0; i <= ns; ++i) {
            double t = i * dt / ns;
            double psit = s.psi + p.omega * t;
            traj.push_back({s.x + iw*(p.vx_b*(std::sin(psit)-spsi) + p.vy_b*(std::cos(psit)-cpsi)),
                            s.y + iw*(p.vx_b*(cpsi-std::cos(psit)) + p.vy_b*(std::sin(psit)-spsi)),
                            s.z + p.vz*t});
        }
        return {nx, ny, nz, npsi};
    }
}

inline bool HybridAStarPlanner::collision_free_trajectory(
    const std::vector<Vec3>& traj, const OccupancyGrid& grid) const {
    for (const auto& pt : traj) {
        auto idx = grid.world_to_index(pt);
        if (grid.is_occupied(idx[0], idx[1], idx[2])) return false;
    }
    return true;
}

inline HybridAStarPlanner::StateKey HybridAStarPlanner::state_key(
    const Vec3& pos, double psi, const OccupancyGrid& grid) const {
    auto idx = grid.world_to_index(pos);
    int hb = static_cast<int>(std::round(psi / heading_res_)) % n_heading_bins_;
    if (hb < 0) hb += n_heading_bins_;
    return {idx[0], idx[1], idx[2], hb};
}

inline std::unordered_map<HybridAStarPlanner::StateKey, double,
                          HybridAStarPlanner::KeyHash, HybridAStarPlanner::KeyEq>
HybridAStarPlanner::precompute_holonomic_heuristic(
    const Vec3& goal, const OccupancyGrid& grid) const {

    std::unordered_map<StateKey, double, KeyHash, KeyEq> cmap;
    auto gidx = grid.world_to_index(goal);
    StateKey gk{gidx[0], gidx[1], gidx[2], 0};
    if (grid.is_occupied(gidx[0], gidx[1], gidx[2])) return cmap;

    double res = grid.resolution;
    cmap[gk] = 0.0;
    using PQEntry = std::pair<double, StateKey>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;
    pq.push({0.0, gk});

    while (!pq.empty()) {
        auto [d, cur] = pq.top(); pq.pop();
        if (d > cmap[cur]) continue;
        int cx = cur[0], cy = cur[1], cz = cur[2];

        for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) continue;
            int nx = cx + dx, ny = cy + dy, nz = cz + dz;
            if (nx < 0 || nx >= grid.nx || ny < 0 || ny >= grid.ny || nz < 0 || nz >= grid.nz) continue;
            if (!grid.is_free(nx, ny, nz)) continue;
            StateKey nk{nx, ny, nz, 0};
            double ec = std::sqrt(dx*dx + dy*dy + dz*dz) * res;
            // Check extra_cost
            auto* cag = dynamic_cast<const CostAwareGrid*>(&grid);
            if (cag) ec += cag->extra_cost(nx, ny, nz);
            double nd = d + ec;
            auto it = cmap.find(nk);
            if (it == cmap.end() || nd < it->second) {
                cmap[nk] = nd;
                pq.push({nd, nk});
            }
        }
    }
    return cmap;
}

inline double HybridAStarPlanner::heuristic(
    const State& s, const Vec3& goal,
    const std::unordered_map<StateKey, double, KeyHash, KeyEq>& holo) const {

    auto idx = grid_->world_to_index({s.x, s.y, s.z});
    StateKey hk{idx[0], idx[1], idx[2], 0};
    double h_holo = std::numeric_limits<double>::infinity();
    auto it = holo.find(hk);
    if (it != holo.end()) h_holo = it->second;

    double dx = s.x - goal.x, dy = s.y - goal.y, dz = s.z - goal.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    double dpsi = std::abs(std::fmod(goal_psi_ - s.psi + kPi, 2.0*kPi) - kPi);
    double h_nh = dist / v_max_ + dpsi / omega_max_;

    return std::max(h_holo, h_nh);
}

inline HybridAStarPlanner::State HybridAStarPlanner::analytic_expand(
    const State& s, const Vec3& goal, const OccupancyGrid& grid) const {

    double dx = goal.x - s.x, dy = goal.y - s.y, dz = goal.z - s.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (dist < 1e-6) return {goal.x, goal.y, goal.z, goal_psi_};

    double step = grid.resolution * 0.5;
    int n = std::max(2, static_cast<int>(dist / step));
    for (int i = 1; i <= n; ++i) {
        double t = static_cast<double>(i) / n;
        Vec3 pt{s.x + dx*t, s.y + dy*t, s.z + dz*t};
        auto idx = grid.world_to_index(pt);
        if (grid.is_occupied(idx[0], idx[1], idx[2])) return {0,0,0,-1};  // invalid
    }
    return {goal.x, goal.y, goal.z, goal_psi_};
}

inline double HybridAStarPlanner::primitive_cost(int pi, int prev_pi) const {
    double cost = dt_;
    if (primitives_[pi].vx_b < -1e-9) cost += reverse_penalty_;
    if (prev_pi >= 0) {
        const auto& pc = primitives_[pi];
        const auto& pp = primitives_[prev_pi];
        double nc = std::sqrt(pc.vx_b*pc.vx_b + pc.vy_b*pc.vy_b + pc.vz*pc.vz);
        double np = std::sqrt(pp.vx_b*pp.vx_b + pp.vy_b*pp.vy_b + pp.vz*pp.vz);
        if (nc > 1e-9 && np > 1e-9) {
            double ca = (pc.vx_b*pp.vx_b + pc.vy_b*pp.vy_b + pc.vz*pp.vz) / (nc * np);
            ca = std::max(-1.0, std::min(1.0, ca));
            if (std::acos(ca) > kPi / 4.0) cost += dir_change_penalty_;
        }
    }
    return cost;
}

inline HybridAStarResult HybridAStarPlanner::plan(
    const Vec3& start, const Vec3& goal, const OccupancyGrid& grid, int seed) {
    (void)seed;

    HybridAStarResult result;
    grid_ = &grid;
    goal_pos_ = goal;

    auto sidx = grid.world_to_index(start);
    auto gidx = grid.world_to_index(goal);
    if (grid.is_occupied(sidx[0], sidx[1], sidx[2]) ||
        grid.is_occupied(gidx[0], gidx[1], gidx[2])) return result;

    double dx = goal.x - start.x, dy = goal.y - start.y;
    double start_psi = (std::abs(dx) > 1e-6 || std::abs(dy) > 1e-6)
                       ? std::atan2(dy, dx) : 0.0;
    goal_psi_ = start_psi;

    State start_state{start.x, start.y, start.z, start_psi};
    auto holo = precompute_holonomic_heuristic(goal, grid);

    using ParentMap = std::unordered_map<StateKey, std::pair<StateKey, int>, KeyHash, KeyEq>;
    using GScoreMap = std::unordered_map<StateKey, double, KeyHash, KeyEq>;
    using VisitedMap = std::unordered_map<StateKey, bool, KeyHash, KeyEq>;

    ParentMap parent;
    GScoreMap g_score;
    VisitedMap visited;

    auto sk = state_key(start, start_psi, grid);
    g_score[sk] = 0.0;
    parent[sk] = {StateKey{-1,-1,-1,-1}, -1};

    double h0 = heuristic(start_state, goal, holo);
    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> pq;
    pq.push({h0, 0.0, sk});

    for (int iter = 0; iter < max_iter_; ++iter) {
        if (pq.empty()) break;
        auto [f, g, ck] = pq.top(); pq.pop();
        if (visited[ck]) continue;
        visited[ck] = true;

        State cs = reconstruct_state(ck, parent, start_state, grid);

        // Termination check
        double pe = std::sqrt((cs.x-goal.x)*(cs.x-goal.x) + (cs.y-goal.y)*(cs.y-goal.y) + (cs.z-goal.z)*(cs.z-goal.z));
        double psie = std::abs(std::fmod(goal_psi_ - cs.psi + kPi, 2.0*kPi) - kPi);
        if (pe < grid.resolution * 1.5 && psie < heading_res_) {
            result.path = extract_path(ck, parent, start_state, grid);
            result.success = true;
            return result;
        }

        // Analytic expansion
        if (iter > 0 && iter % analytic_interval_ == 0) {
            auto gs = analytic_expand(cs, goal, grid);
            if (gs.z >= 0) {
                Vec3 gp{gs.x, gs.y, gs.z};
                auto gk = state_key(gp, gs.psi, grid);
                parent[gk] = {ck, -2};
                g_score[gk] = g_score[ck] + pe / v_max_;
                result.path = extract_path(gk, parent, start_state, grid);
                result.success = true;
                return result;
            }
        }

        // Expand motion primitives
        int prev_pi = parent[ck].second;
        for (int pi = 0; pi < static_cast<int>(primitives_.size()); ++pi) {
            std::vector<Vec3> traj;
            State ns = propagate(cs, pi, traj);
            if (!collision_free_trajectory(traj, grid)) continue;

            Vec3 np{ns.x, ns.y, ns.z};
            auto nidx = grid.world_to_index(np);
            if (nidx[0] < 0 || nidx[0] >= grid.nx ||
                nidx[1] < 0 || nidx[1] >= grid.ny ||
                nidx[2] < 0 || nidx[2] >= grid.nz) continue;

            auto nk = state_key(np, ns.psi, grid);
            if (visited[nk]) continue;

            double ac = primitive_cost(pi, prev_pi);
            double ng = g_score[ck] + ac;
            auto it = g_score.find(nk);
            if (it == g_score.end() || ng < it->second) {
                g_score[nk] = ng;
                parent[nk] = {ck, pi};
                double h = heuristic(ns, goal, holo);
                pq.push({ng + h, ng, nk});
            }
        }
    }

    return result;
}

inline HybridAStarPlanner::State HybridAStarPlanner::reconstruct_state(
    const StateKey& key, const ParentMap& parent,
    const State& start_state, const OccupancyGrid& grid) const {

    auto it = parent.find(key);
    if (it == parent.end()) return start_state;
    if (it->second.second == -1) return start_state;

    auto [pk, pi] = it->second;
    State ps = reconstruct_state(pk, parent, start_state, grid);
    std::vector<Vec3> traj;
    if (pi >= 0) return propagate(ps, pi, traj);
    // pi == -2: analytic expansion
    return {goal_pos_.x, goal_pos_.y, goal_pos_.z, goal_psi_};
}

inline std::vector<Vec3> HybridAStarPlanner::extract_path(
    const StateKey& gk, const ParentMap& parent,
    const State& start_state, const OccupancyGrid& grid) const {

    std::vector<Vec3> path;
    StateKey key = gk;
    while (true) {
        State s = reconstruct_state(key, parent, start_state, grid);
        path.push_back({s.x, s.y, s.z});
        auto it = parent.find(key);
        if (it == parent.end()) break;
        if (it->second.second == -1) break;  // start sentinel only
        key = it->second.first;
    }
    std::reverse(path.begin(), path.end());

    // Downsample
    double spacing = std::max(grid.resolution * 2.5, v_max_ * dt_);
    return downsample_path(path, spacing);
}

inline std::vector<Vec3> HybridAStarPlanner::downsample_path(
    const std::vector<Vec3>& path, double spacing) const {
    if (path.size() < 2) return path;
    std::vector<double> seg_lens;
    seg_lens.reserve(path.size());
    seg_lens.push_back(0);
    for (size_t i = 1; i < path.size(); ++i)
        seg_lens.push_back(seg_lens.back() + norm(path[i] - path[i-1]));
    double total = seg_lens.back();
    if (total < spacing) return path;

    int n = std::max(2, static_cast<int>(std::ceil(total / spacing)));
    std::vector<Vec3> out(n);
    for (int i = 0; i < n; ++i) {
        double target = static_cast<double>(i) / (n - 1) * total;
        auto it = std::lower_bound(seg_lens.begin(), seg_lens.end(), target);
        size_t j = std::min(static_cast<size_t>(it - seg_lens.begin()), seg_lens.size() - 1);
        if (j == 0) { out[i] = path[0]; continue; }
        double seg = seg_lens[j] - seg_lens[j-1];
        double frac = (seg > 1e-12) ? (target - seg_lens[j-1]) / seg : 0.0;
        frac = std::max(0.0, std::min(1.0, frac));
        out[i] = path[j-1] + (path[j] - path[j-1]) * frac;
    }
    return out;
}

}  // namespace sim
