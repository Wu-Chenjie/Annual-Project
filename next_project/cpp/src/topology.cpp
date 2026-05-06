#include "topology.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace sim {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSigmaSquared = 4.0;

double offdiag_norm_sq(const std::vector<std::vector<double>>& m) {
    double acc = 0.0;
    for (std::size_t i = 0; i < m.size(); ++i) {
        for (std::size_t j = 0; j < m[i].size(); ++j) {
            if (i == j) continue;
            acc += m[i][j] * m[i][j];
        }
    }
    return acc;
}

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
    auto [lateral, longitudinal, vertical] = envelope_per_axis(formation);
    return std::max({lateral, longitudinal, vertical});
}

std::tuple<double, double, double> FormationTopology::envelope_per_axis(const std::string& formation) const {
    const auto name = formation.empty()
        ? (switching_ ? target_formation_ : current_formation_)
        : formation;
    const auto offsets = get_offsets(name);
    if (offsets.empty()) {
        return {arm_length_, arm_length_, arm_length_};
    }
    double max_abs_x = 0.0;
    double max_abs_y = 0.0;
    double max_abs_z = 0.0;
    for (const auto& off : offsets) {
        max_abs_x = std::max(max_abs_x, std::abs(off.x));
        max_abs_y = std::max(max_abs_y, std::abs(off.y));
        max_abs_z = std::max(max_abs_z, std::abs(off.z));
    }
    return {
        max_abs_y + arm_length_,
        max_abs_x + arm_length_,
        max_abs_z + arm_length_,
    };
}

bool FormationTopology::auto_shrink(const std::tuple<double, double, double>& channel_width,
                                    const std::tuple<double, double, double>* envelope) {
    const auto env = envelope ? *envelope : envelope_per_axis(current_formation_);
    const bool too_narrow =
        std::get<0>(channel_width) < 2.0 * std::get<0>(env)
        || std::get<1>(channel_width) < 2.0 * std::get<1>(env)
        || std::get<2>(channel_width) < 2.0 * std::get<2>(env);
    const bool roomy =
        std::get<0>(channel_width) > 3.0 * std::get<0>(env)
        && std::get<1>(channel_width) > 3.0 * std::get<1>(env)
        && std::get<2>(channel_width) > 3.0 * std::get<2>(env);
    if (too_narrow && current_formation_ != "line") {
        switch_formation("line", 1.5, 0.0);
        current_offsets_ = target_offsets_;
        current_formation_ = target_formation_;
        switching_ = false;
        return true;
    }
    if (roomy && current_formation_ == "line") {
        switch_formation("diamond", 2.0, 0.0);
        current_offsets_ = target_offsets_;
        current_formation_ = target_formation_;
        switching_ = false;
        return true;
    }
    return false;
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
    std::vector<std::vector<double>> laplacian(static_cast<std::size_t>(n), std::vector<double>(static_cast<std::size_t>(n), 0.0));

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const Vec3 delta = positions[i] - positions[j];
            const double d2 = dot(delta, delta);
            const double w = std::exp(-d2 / kSigmaSquared);
            laplacian[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] += w;
            laplacian[static_cast<std::size_t>(j)][static_cast<std::size_t>(j)] += w;
            laplacian[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] -= w;
            laplacian[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] -= w;
        }
    }

    const int max_iter = 64;
    for (int iter = 0; iter < max_iter && offdiag_norm_sq(laplacian) > 1e-18; ++iter) {
        int p = 0;
        int q = 1;
        double max_val = 0.0;
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                const double v = std::abs(laplacian[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
                if (v > max_val) {
                    max_val = v;
                    p = i;
                    q = j;
                }
            }
        }
        if (max_val < 1e-12) {
            break;
        }

        const double app = laplacian[static_cast<std::size_t>(p)][static_cast<std::size_t>(p)];
        const double aqq = laplacian[static_cast<std::size_t>(q)][static_cast<std::size_t>(q)];
        const double apq = laplacian[static_cast<std::size_t>(p)][static_cast<std::size_t>(q)];
        const double tau = (aqq - app) / (2.0 * apq);
        const double t = ((tau >= 0.0) ? 1.0 : -1.0) / (std::abs(tau) + std::sqrt(1.0 + tau * tau));
        const double c = 1.0 / std::sqrt(1.0 + t * t);
        const double s = t * c;

        for (int k = 0; k < n; ++k) {
            if (k == p || k == q) continue;
            const double aik = laplacian[static_cast<std::size_t>(k)][static_cast<std::size_t>(p)];
            const double akq = laplacian[static_cast<std::size_t>(k)][static_cast<std::size_t>(q)];
            laplacian[static_cast<std::size_t>(k)][static_cast<std::size_t>(p)] = c * aik - s * akq;
            laplacian[static_cast<std::size_t>(p)][static_cast<std::size_t>(k)] = laplacian[static_cast<std::size_t>(k)][static_cast<std::size_t>(p)];
            laplacian[static_cast<std::size_t>(k)][static_cast<std::size_t>(q)] = s * aik + c * akq;
            laplacian[static_cast<std::size_t>(q)][static_cast<std::size_t>(k)] = laplacian[static_cast<std::size_t>(k)][static_cast<std::size_t>(q)];
        }

        const double new_app = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        const double new_aqq = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        laplacian[static_cast<std::size_t>(p)][static_cast<std::size_t>(p)] = new_app;
        laplacian[static_cast<std::size_t>(q)][static_cast<std::size_t>(q)] = new_aqq;
        laplacian[static_cast<std::size_t>(p)][static_cast<std::size_t>(q)] = 0.0;
        laplacian[static_cast<std::size_t>(q)][static_cast<std::size_t>(p)] = 0.0;
    }

    std::vector<double> eigenvals;
    eigenvals.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        eigenvals.push_back(laplacian[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)]);
    }
    std::sort(eigenvals.begin(), eigenvals.end());
    return eigenvals.size() >= 2 ? eigenvals[1] : 0.0;
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
