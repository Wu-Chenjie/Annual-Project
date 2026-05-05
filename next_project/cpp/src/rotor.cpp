#include "rotor.hpp"

namespace sim {

void Rotor::update(double omega_cmd, double dt) {
    omega_cmd = std::clamp(omega_cmd, 0.0, omega_max_);

    if (tau_motor_ > 1e-9) {
        const double alpha = 1.0 - std::exp(-dt / tau_motor_);
        omega_ += (omega_cmd - omega_) * alpha;
    } else {
        omega_ = omega_cmd;
    }

    omega_ = std::clamp(omega_, 0.0, omega_max_);
}

}  // namespace sim
