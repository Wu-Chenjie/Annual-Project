#pragma once

#include <array>
#include <cmath>
#include <unordered_map>

#include "math_utils.hpp"

namespace sim {

class FaultDetector {
public:
    FaultDetector(double max_acc = 10.0, double pos_dev_threshold = 5.0,
                  int saturate_steps = 50, double dt = 0.012)
        : max_acc_(max_acc),
          pos_dev_threshold_(pos_dev_threshold),
          saturate_steps_(saturate_steps),
          dt_(dt) {}

    [[nodiscard]] bool check(int agent_id, const std::array<double, 12>& state,
                             const Vec3& desired_position,
                             const Vec3& desired_velocity,
                             const std::array<double, 4>& control,
                             double control_max = 10.0) {
        Vec3 vel{state[3], state[4], state[5]};
        Vec3 pos{state[0], state[1], state[2]};

        bool acc_detected = false;
        auto prev_it = prev_vel_.find(agent_id);
        if (prev_it != prev_vel_.end()) {
            double acc_mag = norm(vel - prev_it->second) / dt_;
            if (acc_mag > max_acc_) {
                acc_duration_[agent_id] += dt_;
                acc_detected = acc_duration_[agent_id] >= 0.5;
            } else {
                acc_duration_[agent_id] = 0.0;
            }
        }
        prev_vel_[agent_id] = vel;

        double pos_dev = norm(pos - desired_position);
        if (pos_dev > pos_dev_threshold_) {
            pos_dev_duration_[agent_id] += dt_;
        } else {
            pos_dev_duration_[agent_id] = 0.0;
        }
        bool pos_detected = pos_dev_duration_[agent_id] >= 1.0;

        double control_norm = 0.0;
        for (double v : control) control_norm += v * v;
        control_norm = std::sqrt(control_norm);
        if (control_norm > 0.95 * control_max) {
            saturate_count_[agent_id] += 1;
        } else {
            saturate_count_[agent_id] = 0;
        }
        bool sat_detected = saturate_count_[agent_id] >= saturate_steps_;

        (void)desired_velocity;
        return acc_detected || pos_detected || sat_detected;
    }

    void reset(int agent_id) {
        prev_vel_.erase(agent_id);
        acc_duration_.erase(agent_id);
        pos_dev_duration_.erase(agent_id);
        saturate_count_.erase(agent_id);
    }

    void reset_all() {
        prev_vel_.clear();
        acc_duration_.clear();
        pos_dev_duration_.clear();
        saturate_count_.clear();
    }

private:
    double max_acc_;
    double pos_dev_threshold_;
    int saturate_steps_;
    double dt_;
    std::unordered_map<int, Vec3> prev_vel_;
    std::unordered_map<int, double> acc_duration_;
    std::unordered_map<int, double> pos_dev_duration_;
    std::unordered_map<int, int> saturate_count_;
};

}  // namespace sim
