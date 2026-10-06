#include "allocator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim {

ControlAllocator::ControlAllocator(double arm_length, double kf, double km)
    : arm_length_(arm_length), kf_(kf), km_(km) {
    const double gamma = km_ / kf_;

    B_ = {{
        {{1.0, 1.0, 1.0, 1.0}},
        {{arm_length_, -arm_length_, -arm_length_, arm_length_}},
        {{arm_length_, arm_length_, -arm_length_, -arm_length_}},
        {{-gamma, gamma, -gamma, gamma}},
    }};

    B_inv_ = invert_4x4(B_);
}

std::array<double, 4> ControlAllocator::allocate_thrusts(const std::array<double, 4>& u) const {
    std::array<double, 4> thrusts{};
    for (std::size_t row = 0; row < 4; ++row) {
        double sum = 0.0;
        for (std::size_t col = 0; col < 4; ++col) {
            sum += B_inv_[row][col] * u[col];
        }
        thrusts[row] = std::max(0.0, sum);
    }
    return thrusts;
}

std::array<double, 4> ControlAllocator::thrusts_to_omegas(const std::array<double, 4>& thrusts) const {
    std::array<double, 4> omegas{};
    for (std::size_t i = 0; i < 4; ++i) {
        const double thrust = std::max(0.0, thrusts[i]);
        omegas[i] = std::sqrt(thrust / kf_);
    }
    return omegas;
}

std::array<double, 4> ControlAllocator::omegas_to_u(const std::array<double, 4>& omegas) const {
    std::array<double, 4> thrusts{};
    for (std::size_t i = 0; i < 4; ++i) {
        thrusts[i] = kf_ * omegas[i] * omegas[i];
    }

    std::array<double, 4> u{};
    for (std::size_t row = 0; row < 4; ++row) {
        double sum = 0.0;
        for (std::size_t col = 0; col < 4; ++col) {
            sum += B_[row][col] * thrusts[col];
        }
        u[row] = sum;
    }
    return u;
}

std::array<std::array<double, 4>, 4> ControlAllocator::invert_4x4(const std::array<std::array<double, 4>, 4>& m) {
    std::array<std::array<double, 8>, 4> aug{};

    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            aug[i][j] = m[i][j];
            aug[i][j + 4] = (i == j ? 1.0 : 0.0);
        }
    }

    for (std::size_t col = 0; col < 4; ++col) {
        std::size_t pivot_row = col;
        double pivot_abs = std::abs(aug[col][col]);
        for (std::size_t row = col + 1; row < 4; ++row) {
            const double candidate = std::abs(aug[row][col]);
            if (candidate > pivot_abs) {
                pivot_abs = candidate;
                pivot_row = row;
            }
        }

        if (pivot_abs < 1e-12) {
            throw std::runtime_error("Control allocation matrix is singular");
        }

        if (pivot_row != col) {
            std::swap(aug[pivot_row], aug[col]);
        }

        const double pivot = aug[col][col];
        for (std::size_t j = 0; j < 8; ++j) {
            aug[col][j] /= pivot;
        }

        for (std::size_t row = 0; row < 4; ++row) {
            if (row == col) {
                continue;
            }
            const double factor = aug[row][col];
            for (std::size_t j = 0; j < 8; ++j) {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    std::array<std::array<double, 4>, 4> inv{};
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            inv[i][j] = aug[i][j + 4];
        }
    }
    return inv;
}

}  // namespace sim
