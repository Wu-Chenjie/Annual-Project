#pragma once

#include <algorithm>
#include <cmath>

namespace sim {

class Rotor {
public:
    Rotor(double kf = 1e-5, double km = 2e-7, int direction = 1,
          double tau_motor = 0.02, double omega_max = 1000.0)
        : kf_(kf),
          km_(km),
          direction_(direction >= 0 ? 1 : -1),
          tau_motor_(tau_motor),
          omega_max_(omega_max) {}

    void update(double omega_cmd, double dt);

    [[nodiscard]] double thrust() const { return kf_ * omega_ * omega_; }
    [[nodiscard]] double torque() const { return static_cast<double>(direction_) * km_ * omega_ * omega_; }

    [[nodiscard]] int direction() const { return direction_; }
    [[nodiscard]] double omega() const { return omega_; }
    [[nodiscard]] double kf() const { return kf_; }
    [[nodiscard]] double km() const { return km_; }

private:
    double kf_;
    double km_;
    int direction_;
    double tau_motor_;
    double omega_max_;
    double omega_ = 0.0;
};

}  // namespace sim
