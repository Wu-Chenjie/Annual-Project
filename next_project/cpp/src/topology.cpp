#include "topology.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace sim {

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

FormationTopology::FormationTopology(int num_followers, double spacing, double arm_length)
    : num_followers_(num_followers), spacing_(spacing), arm_length_(arm_length) {
    if (num_followers_ < 1) {
        throw std::invalid_argument("num_followers must be >= 1");
    }
    if (spacing_ <= 0.0) {
        throw std::invalid_argument("spacing must be positive");
    }

    current_offsets_ = compute_offsets("v_shape");
}

std::vector<Vec3> FormationTopology::compute_v_shape() const {
    std::vector<Vec3> offsets;
    offsets.reserve(static_cast<std::size_t>(num_followers_));

    for (int i = 0; i < num_followers_; ++i) {
        const int rank = (i / 2) + 1;
        const int side = (i % 2 == 0) ? -1 : 1;
        offsets.emplace_back(-rank * spacing_, side * rank * spacing_, 0.0);
    }
    return offsets;
}

std::vector<Vec3> FormationTopology::compute_diamond() const {
    std::vector<Vec3> offsets{
        Vec3{-spacing_, -spacing_, 0.0},
        Vec3{-spacing_, spacing_, 0.0},
        Vec3{-2.0 * spacing_, 0.0, 0.0},
    };

    while (static_cast<int>(offsets.size()) < num_followers_) {
        const int extra_rank = static_cast<int>(offsets.size()) - 2;
        offsets.emplace_back(-2.0 * spacing_ - extra_rank * spacing_, 0.0, 0.0);
    }

    offsets.resize(static_cast<std::size_t>(num_followers_));
    return offsets;
}

std::vector<Vec3> FormationTopology::compute_line() const {
    std::vector<Vec3> offsets;
    offsets.reserve(static_cast<std::size_t>(num_followers_));

    for (int i = 0; i < num_followers_; ++i) {
        offsets.emplace_back(-(i + 1) * spacing_, 0.0, 0.0);
    }

    return offsets;
}

std::vector<Vec3> FormationTopology::compute_triangle() const {
    std::vector<Vec3> offsets;
    offsets.reserve(static_cast<std::size_t>(num_followers_));

    const double row_height = spacing_ * std::sqrt(3.0) / 2.0;
    int placed = 0;
    int row = 1;

    while (placed < num_followers_) {
        const int num_in_row = row + 1;
        const double row_y_start = -row * spacing_ / 2.0;

        for (int col = 0; col < num_in_row && placed < num_followers_; ++col) {
            const double dx = -row * row_height;
            const double dy = row_y_start + col * spacing_;
            offsets.emplace_back(dx, dy, 0.0);
            ++placed;
        }

        ++row;
    }

    return offsets;
}

std::vector<Vec3> FormationTopology::compute_offsets(const std::string& formation) const {
    if (formation == "v_shape") {
        return compute_v_shape();
    }
    if (formation == "diamond") {
        return compute_diamond();
    }
    if (formation == "line") {
        return compute_line();
    }
    if (formation == "triangle") {
        return compute_triangle();
    }
    if (formation == "custom") {
        if (custom_offsets_.empty()) {
            throw std::invalid_argument("custom formation requires set_custom_offsets first");
        }
        if (static_cast<int>(custom_offsets_.size()) != num_followers_) {
            throw std::invalid_argument("custom offsets count does not match followers");
        }
        return custom_offsets_;
    }

    throw std::invalid_argument("unknown formation: " + formation);
}

std::vector<Vec3> FormationTopology::get_offsets(const std::string& formation) const {
    return compute_offsets(formation);
}

void FormationTopology::set_custom_offsets(const std::vector<Vec3>& offsets) {
    if (static_cast<int>(offsets.size()) != num_followers_) {
        throw std::invalid_argument("offset count does not match followers");
    }
    custom_offsets_ = offsets;
}

void FormationTopology::switch_formation(const std::string& target_formation, double transition_time, double current_time) {
    if (transition_time <= 0.0) {
        throw std::invalid_argument("transition_time must be positive");
    }

    source_offsets_ = current_offsets_;
    target_offsets_ = compute_offsets(target_formation);
    target_formation_ = target_formation;
    transition_time_ = transition_time;
    switch_start_time_ = 0.0;  // 对齐 Python：瞬时切换
    switching_ = true;
}

std::vector<Vec3> FormationTopology::get_current_offsets(double elapsed_time) {
    if (!switching_) {
        return current_offsets_;
    }

    const double t = elapsed_time - switch_start_time_;
    if (t >= transition_time_) {
        current_offsets_ = target_offsets_;
        current_formation_ = target_formation_;
        switching_ = false;
        return current_offsets_;
    }

    const double alpha = 0.5 * (1.0 - std::cos(kPi * t / transition_time_));

    std::vector<Vec3> blended;
    blended.reserve(current_offsets_.size());

    for (std::size_t i = 0; i < source_offsets_.size(); ++i) {
        blended.push_back((1.0 - alpha) * source_offsets_[i] + alpha * target_offsets_[i]);
    }

    return blended;
}

double FormationTopology::envelope_radius(const std::string& formation) const {
    const auto offsets = get_offsets(formation);
    double max_dist = 0.0;
    for (const auto& off : offsets) {
        const double d = norm(off);
        if (d > max_dist) max_dist = d;
    }
    return max_dist + arm_length_;
}

std::string FormationTopology::fault_reconfigure(const std::vector<int>& failed_indices,
                                                 double transition_time,
                                                 double current_time) {
    std::string best = TopologyGraph::best_reconfig_topology(
        failed_indices, num_followers_, spacing_);
    if (!best.empty()) {
        switch_formation(best, transition_time, current_time);
    }
    return best;
}

TopologyGraph::TopologyGraph(const std::vector<Vec3>& offsets)
    : offsets_(offsets) {}

double TopologyGraph::algebraic_connectivity() const {
    if (offsets_.empty()) return 0.0;
    std::vector<Vec3> positions{Vec3{}};
    positions.insert(positions.end(), offsets_.begin(), offsets_.end());
    const int n = static_cast<int>(positions.size());

    double min_degree = std::numeric_limits<double>::infinity();
    for (int i = 0; i < n; ++i) {
        double degree = 0.0;
        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            double d = norm(positions[i] - positions[j]);
            degree += std::exp(-(d * d) / 4.0);
        }
        min_degree = std::min(min_degree, degree);
    }
    return std::isfinite(min_degree) ? min_degree : 0.0;
}

std::string TopologyGraph::best_reconfig_topology(const std::vector<int>& failed_indices,
                                                  int num_followers,
                                                  double spacing,
                                                  const std::vector<std::string>& candidates) {
    int healthy = num_followers - static_cast<int>(failed_indices.size());
    if (healthy < 1) return "";

    std::string best;
    double best_score = -1.0;
    for (const auto& formation : candidates) {
        auto offsets = compute_offsets_for(formation, healthy, spacing);
        if (static_cast<int>(offsets.size()) != healthy) continue;
        TopologyGraph graph(offsets);
        double score = graph.algebraic_connectivity();
        if (score > best_score) {
            best_score = score;
            best = formation;
        }
    }
    return best;
}

std::vector<Vec3> TopologyGraph::compute_offsets_for(const std::string& formation,
                                                     int num_followers,
                                                     double spacing) {
    FormationTopology topo(num_followers, spacing);
    return topo.get_offsets(formation);
}

}  // namespace sim
