#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <vector>

#include "math_utils.hpp"
#include "occupancy_grid.hpp"

namespace sim {

class DStarLite {
public:
    DStarLite(const OccupancyGrid& grid, const Vec3& start, const Vec3& goal);

    void compute_shortest_path();
    void update_start(const Vec3& new_start);
    void update_cells(const std::vector<std::pair<std::array<int,3>, bool>>& changed);
    std::vector<Vec3> extract_path() const;
    [[nodiscard]] bool is_consistent() const;

private:
    using Idx = std::array<int, 3>;
    struct IdxHash { std::size_t operator()(const Idx& k) const {
        return (std::size_t(k[0])*73856093)^(std::size_t(k[1])*19349663)^(std::size_t(k[2])*83492791); }};
    struct IdxEq { bool operator()(const Idx& a, const Idx& b) const { return a[0]==b[0]&&a[1]==b[1]&&a[2]==b[2]; }};

    double heuristic(const Idx& a, const Idx& b) const;
    double cost(const Idx& a, const Idx& b) const;
    std::pair<double,double> calculate_key(const Idx& s) const;
    void insert(const Idx& s);
    void update_vertex(const Idx& u);
    std::vector<Idx> successors(const Idx& idx) const;
    Idx pop_valid();
    std::pair<double,double> top_key() const;
    void gc_heap();

    const OccupancyGrid* grid_;
    std::unordered_map<Idx, double, IdxHash, IdxEq> g_, rhs_;
    using PQEntry = std::tuple<double, double, int, Idx>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> U_;
    std::unordered_map<Idx, int, IdxHash, IdxEq> U_version_;
    double k_m_ = 0.0;
    Idx start_idx_, goal_idx_, last_start_idx_;
    Vec3 start_, goal_;
    int step_counter_ = 0;
};

// Implementation
inline DStarLite::DStarLite(const OccupancyGrid& grid, const Vec3& start, const Vec3& goal)
    : grid_(&grid), start_(start), goal_(goal) {
    start_idx_ = grid.world_to_index(start);
    goal_idx_ = grid.world_to_index(goal);
    last_start_idx_ = start_idx_;
    rhs_[goal_idx_] = 0.0;
    insert(goal_idx_);
}

inline double DStarLite::heuristic(const Idx& a, const Idx& b) const {
    Vec3 pa = grid_->index_to_world(a[0], a[1], a[2]);
    Vec3 pb = grid_->index_to_world(b[0], b[1], b[2]);
    return norm(pa - pb);
}

inline double DStarLite::cost(const Idx& a, const Idx& b) const {
    if (grid_->is_occupied(a[0],a[1],a[2]) || grid_->is_occupied(b[0],b[1],b[2]))
        return std::numeric_limits<double>::infinity();
    return heuristic(a, b);
}

inline std::pair<double,double> DStarLite::calculate_key(const Idx& s) const {
    double gm = std::min(g_.count(s) ? g_.at(s) : std::numeric_limits<double>::infinity(),
                         rhs_.count(s) ? rhs_.at(s) : std::numeric_limits<double>::infinity());
    return {gm + heuristic(start_idx_, s) + k_m_, gm};
}

inline void DStarLite::insert(const Idx& s) {
    auto [k1, k2] = calculate_key(s);
    int ver = (U_version_.count(s) ? U_version_[s] : 0) + 1;
    U_version_[s] = ver;
    U_.push({k1, k2, ver, s});
}

inline DStarLite::Idx DStarLite::pop_valid() {
    while (!U_.empty()) {
        auto [k1, k2, ver, s] = U_.top(); U_.pop();
        if (U_version_.count(s) && U_version_[s] == ver) {
            U_version_[s] = ver + 1;
            return s;
        }
    }
    return {-1,-1,-1};
}

inline std::pair<double,double> DStarLite::top_key() const {
    auto* self = const_cast<DStarLite*>(this);
    while (!self->U_.empty()) {
        auto [k1, k2, ver, s] = self->U_.top();
        auto it = U_version_.find(s);
        if (it != U_version_.end() && it->second == ver) return {k1, k2};
        self->U_.pop();
    }
    return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
}

inline void DStarLite::gc_heap() {
    std::vector<PQEntry> keep;
    while (!U_.empty()) {
        auto e = U_.top(); U_.pop();
        const Idx& s = std::get<3>(e);
        if (U_version_.count(s) && U_version_[s] == std::get<2>(e)) keep.push_back(e);
    }
    for (auto& e : keep) U_.push(e);
}

inline std::vector<DStarLite::Idx> DStarLite::successors(const Idx& idx) const {
    std::vector<Idx> out;
    for (int d = 0; d < 3; ++d) for (int delta : {-1, 1}) {
        Idx n = idx; n[d] += delta;
        if (n[0]>=0&&n[0]<grid_->nx && n[1]>=0&&n[1]<grid_->ny && n[2]>=0&&n[2]<grid_->nz) out.push_back(n);
    }
    return out;
}

inline void DStarLite::update_vertex(const Idx& u) {
    if (u != goal_idx_) {
        double min_rhs = std::numeric_limits<double>::infinity();
        for (const auto& s : successors(u)) {
            double c = cost(u, s);
            double gs = g_.count(s) ? g_[s] : std::numeric_limits<double>::infinity();
            if (c < std::numeric_limits<double>::infinity()) min_rhs = std::min(min_rhs, c + gs);
        }
        rhs_[u] = min_rhs;
    }
    double gu = g_.count(u) ? g_[u] : std::numeric_limits<double>::infinity();
    if (std::abs(gu - rhs_[u]) > 1e-9) insert(u);
    else if (U_version_.count(u)) U_version_[u] += 1;
}

inline void DStarLite::compute_shortest_path() {
    while (!U_.empty()) {
        auto [k1, k2] = top_key();
        auto sk = calculate_key(start_idx_);
        if (sk < std::make_pair(k1, k2) && std::abs((rhs_.count(start_idx_)?rhs_[start_idx_]:std::numeric_limits<double>::infinity()) - (g_.count(start_idx_)?g_[start_idx_]:std::numeric_limits<double>::infinity())) < 1e-9) break;
        Idx u = pop_valid();
        if (u[0] < 0) break;
        auto ck = calculate_key(u);
        if (std::make_pair(k1,k2) < ck) { insert(u); continue; }
        double gu = g_.count(u) ? g_[u] : std::numeric_limits<double>::infinity();
        double ru = rhs_.count(u) ? rhs_[u] : std::numeric_limits<double>::infinity();
        if (gu > ru) {
            g_[u] = ru;
            for (const auto& pred : successors(u)) update_vertex(pred);
        } else {
            g_[u] = std::numeric_limits<double>::infinity();
            for (const auto& pred : successors(u)) update_vertex(pred);
            update_vertex(u);
        }
    }
    last_start_idx_ = start_idx_;
    if (++step_counter_ % 100 == 0) gc_heap();
}

inline void DStarLite::update_start(const Vec3& new_start) {
    Idx ni = grid_->world_to_index(new_start);
    k_m_ += heuristic(last_start_idx_, ni);
    start_idx_ = ni;
    start_ = new_start;
}

inline void DStarLite::update_cells(const std::vector<std::pair<Idx, bool>>& changed) {
    for (const auto& [idx, occ] : changed)
        const_cast<OccupancyGrid*>(grid_)->data[(idx[2]*grid_->ny+idx[1])*grid_->nx+idx[0]] = occ ? 1 : 0;
    std::unordered_map<Idx, bool, IdxHash, IdxEq> updated;
    for (const auto& [idx, _] : changed) {
        update_vertex(idx); updated[idx] = true;
        for (const auto& pred : successors(idx)) {
            if (!updated[pred]) { update_vertex(pred); updated[pred] = true; }
        }
    }
}

inline std::vector<Vec3> DStarLite::extract_path() const {
    if ((g_.count(start_idx_) ? g_.at(start_idx_) : std::numeric_limits<double>::infinity()) >= std::numeric_limits<double>::infinity() - 1)
        return {};
    std::vector<Vec3> path;
    Idx cur = start_idx_;
    std::unordered_map<Idx, bool, IdxHash, IdxEq> visited;
    for (int step = 0; step < 5000; ++step) {
        path.push_back(grid_->index_to_world(cur[0], cur[1], cur[2]));
        if (cur == goal_idx_) break;
        if (visited[cur]) break;
        visited[cur] = true;
        Idx best; double bc = std::numeric_limits<double>::infinity();
        for (const auto& s : successors(cur)) {
            double cc = cost(cur, s) + (g_.count(s) ? g_.at(s) : std::numeric_limits<double>::infinity());
            if (cc < bc) { bc = cc; best = s; }
        }
        if (bc >= std::numeric_limits<double>::infinity() - 1) break;
        cur = best;
    }
    return path;
}

inline bool DStarLite::is_consistent() const {
    double gu = g_.count(start_idx_) ? g_.at(start_idx_) : std::numeric_limits<double>::infinity();
    double ru = rhs_.count(start_idx_) ? rhs_.at(start_idx_) : std::numeric_limits<double>::infinity();
    return std::abs(gu - ru) < 1e-9;
}

}  // namespace sim
