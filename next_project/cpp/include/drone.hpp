#pragma once

#include <array>

#include "allocator.hpp"
#include "math_utils.hpp"
#include "rotor.hpp"

namespace sim {

struct KinematicState {
    Vec3 position;
    Vec3 velocity;
    Vec3 attitude;
    Vec3 angular_velocity;
};

class Drone {
public:
    Drone(double m = 1.0, const std::array<double, 3>& inertia_diag = {0.01, 0.01, 0.02},
          double arm_length = 0.2, double dt = 0.01);

    void set_initial_state(const Vec3& position, const Vec3& velocity,
                           const Vec3& attitude = Vec3{0.0, 0.0, 0.0},
                           const Vec3& angular_velocity = Vec3{0.0, 0.0, 0.0},
                           double dt_override = -1.0);

    void set_wind(const Vec3& wind);
    void inject_fault(int rotor_index, double severity);
    void clear_fault();
    void update_state(const std::array<double, 4>& control, const Vec3& wind);

    [[nodiscard]] const std::array<double, 12>& state() const { return state_; }
    [[nodiscard]] KinematicState get_state() const;
    [[nodiscard]] double dt() const { return dt_; }
    [[nodiscard]] bool fault_active() const { return fault_active_; }

private:
    std::array<double, 4> rotor_step(const std::array<double, 4>& desired_u);
    std::array<double, 12> dynamics(const std::array<double, 12>& state, const std::array<double, 4>& u, const Vec3& wind) const;

    double m_;
    std::array<double, 3> inertia_diag_;
    std::array<double, 3> inertia_inv_diag_;
    double g_ = 9.81;
    double arm_length_;
    double dt_;
    double k_drag_ = 0.05;
    Vec3 wind_{};
    double j_rotor_ = 3.0e-5;
    std::array<double, 4> fault_mask_{1.0, 1.0, 1.0, 1.0};
    bool fault_active_ = false;

    std::array<double, 12> state_{};
    std::array<Rotor, 4> rotors_;
    ControlAllocator allocator_;
};

class QuaternionDrone {
public:
    QuaternionDrone(double m = 1.0, const std::array<double, 3>& inertia_diag = {0.01, 0.01, 0.02},
                    double arm_length = 0.2, double dt = 0.01);

    void set_initial_state(const Vec3& position, const Vec3& velocity,
                           const Vec3& attitude = Vec3{0.0, 0.0, 0.0},
                           const Vec3& angular_velocity = Vec3{0.0, 0.0, 0.0},
                           double dt_override = -1.0);

    void set_wind(const Vec3& wind);
    void inject_fault(int rotor_index, double severity);
    void clear_fault();
    void update_state(const std::array<double, 4>& control, const Vec3& wind);

    [[nodiscard]] const std::array<double, 13>& state() const { return state_; }
    [[nodiscard]] KinematicState get_state() const;
    [[nodiscard]] double dt() const { return dt_; }
    [[nodiscard]] bool fault_active() const { return fault_active_; }

private:
    std::array<double, 4> rotor_step(const std::array<double, 4>& desired_u);
    std::array<double, 13> dynamics(const std::array<double, 13>& state, const std::array<double, 4>& u, const Vec3& wind) const;

    double m_;
    std::array<double, 3> inertia_diag_;
    std::array<double, 3> inertia_inv_diag_;
    double g_ = 9.81;
    double arm_length_;
    double dt_;
    double k_drag_ = 0.05;
    Vec3 wind_{};
    double j_rotor_ = 3.0e-5;
    std::array<double, 4> fault_mask_{1.0, 1.0, 1.0, 1.0};
    bool fault_active_ = false;

    std::array<double, 13> state_{};
    std::array<Rotor, 4> rotors_;
    ControlAllocator allocator_;
};

}  // namespace sim
