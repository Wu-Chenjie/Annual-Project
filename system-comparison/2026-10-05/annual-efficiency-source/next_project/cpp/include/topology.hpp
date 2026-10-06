#pragma once

#include <string>
#include <tuple>
#include <vector>

#include "math_utils.hpp"

namespace sim {

class FormationTopology {
public:
    FormationTopology(int num_followers = 3, double spacing = 2.0, double arm_length = 0.2);

    std::vector<Vec3> get_offsets(const std::string& formation) const;
    std::vector<Vec3> get_current_offsets(double elapsed_time);

    void set_custom_offsets(const std::vector<Vec3>& offsets);
    void set_current_formation(const std::string& formation);
    void switch_formation(const std::string& target_formation, double transition_time, double current_time);

    [[nodiscard]] bool is_switching() const { return switching_; }
    [[nodiscard]] std::tuple<double, double, double> envelope_per_axis(
        const std::string& formation = "") const;
    [[nodiscard]] double envelope_radius(const std::string& formation) const;
    [[nodiscard]] const std::string& current_formation() const { return current_formation_; }
    bool auto_shrink(const std::tuple<double, double, double>& channel_width,
                     const std::tuple<double, double, double>* envelope = nullptr);
    std::string fault_reconfigure(const std::vector<int>& failed_indices,
                                  double transition_time,
                                  double current_time);

private:
    std::vector<Vec3> compute_offsets(const std::string& formation) const;
    std::vector<Vec3> compute_v_shape() const;
    std::vector<Vec3> compute_diamond() const;
    std::vector<Vec3> compute_line() const;
    std::vector<Vec3> compute_triangle() const;

    int num_followers_;
    double spacing_;
    double arm_length_;
    std::vector<Vec3> current_offsets_;
    std::vector<Vec3> custom_offsets_;
    std::string current_formation_ = "v_shape";

    bool switching_ = false;
    double switch_start_time_ = 0.0;
    double transition_time_ = 0.0;
    std::vector<Vec3> source_offsets_;
    std::vector<Vec3> target_offsets_;
    std::string target_formation_;
};

class TopologyGraph {
public:
    explicit TopologyGraph(const std::vector<Vec3>& offsets);

    [[nodiscard]] double algebraic_connectivity() const;
    [[nodiscard]] static std::string best_reconfig_topology(
        const std::vector<int>& failed_indices,
        int num_followers,
        double spacing,
        const std::vector<std::string>& candidates = {"v_shape", "diamond", "line", "triangle"});

private:
    [[nodiscard]] static std::vector<Vec3> compute_offsets_for(const std::string& formation,
                                                               int num_followers,
                                                               double spacing);
    std::vector<Vec3> offsets_;
};

}  // namespace sim
