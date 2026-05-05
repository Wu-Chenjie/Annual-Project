#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <limits>
#include <variant>
#include <vector>

#include "math_utils.hpp"

namespace sim {

struct AABB {
    Vec3 min_corner;
    Vec3 max_corner;

    [[nodiscard]] double signed_distance(const Vec3& p) const {
        Vec3 half{
            (max_corner.x - min_corner.x) * 0.5,
            (max_corner.y - min_corner.y) * 0.5,
            (max_corner.z - min_corner.z) * 0.5,
        };
        Vec3 q{
            std::abs(p.x - (min_corner.x + max_corner.x) * 0.5) - half.x,
            std::abs(p.y - (min_corner.y + max_corner.y) * 0.5) - half.y,
            std::abs(p.z - (min_corner.z + max_corner.z) * 0.5) - half.z,
        };
        double outside = norm(Vec3{std::max(q.x, 0.0), std::max(q.y, 0.0), std::max(q.z, 0.0)});
        double inside = std::min(std::max({q.x, q.y, q.z}), 0.0);
        return outside + inside;
    }
};

struct Sphere {
    Vec3 center;
    double radius;

    [[nodiscard]] double signed_distance(const Vec3& p) const {
        return norm(p - center) - radius;
    }
};

struct Cylinder {
    Vec3 center_xy;
    double radius;
    double z_min;
    double z_max;

    [[nodiscard]] double signed_distance(const Vec3& p) const {
        double dx = std::sqrt((p.x - center_xy.x) * (p.x - center_xy.x)
                            + (p.y - center_xy.y) * (p.y - center_xy.y)) - radius;
        double dz_bottom = z_min - p.z;
        double dz_top = p.z - z_max;
        double dz = std::max({dz_bottom, dz_top, 0.0});
        if (dz <= 0.0 && dx > 0.0) return dx;
        if (dx <= 0.0 && dz <= 0.0) return std::max({dx, dz_bottom, dz_top});
        return std::sqrt(std::max(dx, 0.0) * std::max(dx, 0.0) + dz * dz);
    }
};

using ObstacleVariant = std::variant<AABB, Sphere, Cylinder>;

class ObstacleField {
public:
    void add_aabb(const Vec3& mn, const Vec3& mx) { add_aabb_id(mn, mx); }
    void add_sphere(const Vec3& center, double r) { add_sphere_id(center, r); }
    void add_cylinder(const Vec3& center_xy, double r, double zmin, double zmax) {
        add_cylinder_id(center_xy, r, zmin, zmax);
    }

    std::string add_aabb_id(const Vec3& mn, const Vec3& mx, const std::string& id = "") {
        obstacles_.emplace_back(AABB{mn, mx});
        ids_.push_back(resolve_id(id));
        return ids_.back();
    }

    std::string add_sphere_id(const Vec3& center, double r, const std::string& id = "") {
        obstacles_.emplace_back(Sphere{center, r});
        ids_.push_back(resolve_id(id));
        return ids_.back();
    }

    std::string add_cylinder_id(const Vec3& center_xy, double r, double zmin, double zmax, const std::string& id = "") {
        obstacles_.emplace_back(Cylinder{center_xy, r, zmin, zmax});
        ids_.push_back(resolve_id(id));
        return ids_.back();
    }

    [[nodiscard]] double signed_distance(const Vec3& p) const {
        if (obstacles_.empty()) return std::numeric_limits<double>::infinity();
        double min_sd = std::numeric_limits<double>::infinity();
        for (const auto& obs : obstacles_) {
            double sd = std::visit([&p](const auto& o) { return o.signed_distance(p); }, obs);
            if (sd < min_sd) min_sd = sd;
        }
        return min_sd;
    }

    [[nodiscard]] bool is_collision(const Vec3& p, double inflate = 0.0) const {
        return signed_distance(p) < inflate;
    }

    [[nodiscard]] std::size_t size() const { return obstacles_.size(); }
    [[nodiscard]] const std::vector<ObstacleVariant>& obstacles() const { return obstacles_; }
    [[nodiscard]] const std::vector<std::string>& ids() const { return ids_; }

    void remove_at(std::size_t index) {
        if (index >= obstacles_.size()) return;
        obstacles_.erase(obstacles_.begin() + static_cast<std::ptrdiff_t>(index));
        if (index < ids_.size()) ids_.erase(ids_.begin() + static_cast<std::ptrdiff_t>(index));
    }

    bool remove_by_id(const std::string& id) {
        auto it = std::find(ids_.begin(), ids_.end(), id);
        if (it == ids_.end()) return false;
        remove_at(static_cast<std::size_t>(std::distance(ids_.begin(), it)));
        return true;
    }

    void clear() {
        obstacles_.clear();
        ids_.clear();
        dyn_counter_ = 0;
    }

    void regenerate_ids(const std::string& prefix = "obs_") {
        ids_.clear();
        ids_.reserve(obstacles_.size());
        for (std::size_t i = 0; i < obstacles_.size(); ++i) {
            ids_.push_back(prefix + std::to_string(i));
        }
    }

private:
    std::string resolve_id(const std::string& id) {
        if (!id.empty()) return id;
        return "dyn_" + std::to_string(dyn_counter_++);
    }

    std::vector<ObstacleVariant> obstacles_;
    std::vector<std::string> ids_;
    int dyn_counter_ = 0;
};

}  // namespace sim
