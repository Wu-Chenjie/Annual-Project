#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace sim {

inline double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(value, upper));
}

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3() = default;
    Vec3(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in) {}

    Vec3& operator+=(const Vec3& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vec3& operator*=(double s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    Vec3& operator/=(double s) {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }

    Vec3 operator-() const { return Vec3{-x, -y, -z}; }
};

inline Vec3 operator+(Vec3 lhs, const Vec3& rhs) {
    lhs += rhs;
    return lhs;
}

inline Vec3 operator-(Vec3 lhs, const Vec3& rhs) {
    lhs -= rhs;
    return lhs;
}

inline Vec3 operator*(Vec3 v, double s) {
    v *= s;
    return v;
}

inline Vec3 operator*(double s, Vec3 v) {
    v *= s;
    return v;
}

inline Vec3 operator/(Vec3 v, double s) {
    v /= s;
    return v;
}

inline double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

inline double norm(const Vec3& v) {
    return std::sqrt(dot(v, v));
}

inline Vec3 normalized(const Vec3& v, const Vec3& fallback = Vec3{0.0, 0.0, 1.0}) {
    const double n = norm(v);
    if (n < 1e-12) {
        return fallback;
    }
    return v / n;
}

inline Vec3 clip_vec(const Vec3& v, double lower, double upper) {
    return Vec3{
        clamp(v.x, lower, upper),
        clamp(v.y, lower, upper),
        clamp(v.z, lower, upper),
    };
}

inline std::array<double, 3> to_array(const Vec3& v) {
    return std::array<double, 3>{v.x, v.y, v.z};
}

inline Vec3 from_array3(const std::array<double, 3>& a) {
    return Vec3{a[0], a[1], a[2]};
}

struct Mat3 {
    std::array<std::array<double, 3>, 3> m{};

    Vec3 operator*(const Vec3& v) const {
        return Vec3{
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
        };
    }
};

inline Mat3 rotation_matrix(double roll, double pitch, double yaw) {
    const double c_r = std::cos(roll);
    const double s_r = std::sin(roll);
    const double c_p = std::cos(pitch);
    const double s_p = std::sin(pitch);
    const double c_y = std::cos(yaw);
    const double s_y = std::sin(yaw);

    Mat3 R{};
    R.m = {{
        {{c_y * c_p, c_y * s_p * s_r - s_y * c_r, c_y * s_p * c_r + s_y * s_r}},
        {{s_y * c_p, s_y * s_p * s_r + c_y * c_r, s_y * s_p * c_r - c_y * s_r}},
        {{-s_p, c_p * s_r, c_p * c_r}},
    }};
    return R;
}

inline std::array<double, 12> state_add_scaled(
    const std::array<double, 12>& base,
    const std::array<double, 12>& delta,
    double scale
) {
    std::array<double, 12> out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = base[i] + scale * delta[i];
    }
    return out;
}

}  // namespace sim
