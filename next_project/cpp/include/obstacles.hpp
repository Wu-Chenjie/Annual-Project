#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <type_traits>
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
    struct Bounds {
        Vec3 min_corner;
        Vec3 max_corner;
    };

public:
    void add_aabb(const Vec3& mn, const Vec3& mx) { add_aabb_id(mn, mx); }
    void add_sphere(const Vec3& center, double r) { add_sphere_id(center, r); }
    void add_cylinder(const Vec3& center_xy, double r, double zmin, double zmax) {
        add_cylinder_id(center_xy, r, zmin, zmax);
    }

    std::string add_aabb_id(const Vec3& mn, const Vec3& mx, const std::string& id = "") {
        obstacles_.emplace_back(AABB{mn, mx});
        ids_.push_back(resolve_id(id));
        invalidate_spatial_index();
        return ids_.back();
    }

    std::string add_sphere_id(const Vec3& center, double r, const std::string& id = "") {
        obstacles_.emplace_back(Sphere{center, r});
        ids_.push_back(resolve_id(id));
        invalidate_spatial_index();
        return ids_.back();
    }

    std::string add_cylinder_id(const Vec3& center_xy, double r, double zmin, double zmax, const std::string& id = "") {
        obstacles_.emplace_back(Cylinder{center_xy, r, zmin, zmax});
        ids_.push_back(resolve_id(id));
        invalidate_spatial_index();
        return ids_.back();
    }

    // 设置外部距离查询回调（如 ESDF），设置后 signed_distance 走 O(1) 回调
    using SDFCallback = double (*)(const void* ctx, double x, double y, double z);
    void set_sdf_callback(SDFCallback cb, const void* ctx) { sdf_cb_ = cb; sdf_ctx_ = ctx; }

    [[nodiscard]] double signed_distance(const Vec3& p) const {
        if (sdf_cb_) return sdf_cb_(sdf_ctx_, p.x, p.y, p.z);  // O(1) 回调
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

    [[nodiscard]] std::vector<const ObstacleVariant*> obstacles_in_ray_window(
        const Vec3& origin, const Vec3& direction, double max_range, double padding = 1e-9) const {
        std::vector<const ObstacleVariant*> out;
        if (obstacles_.empty()) return out;

        const double dir_norm = norm(direction);
        if (dir_norm <= 1e-12 || max_range <= 0.0) return out;
        const Vec3 dir = direction / dir_norm;
        const Vec3 end = origin + dir * max_range;
        Bounds window{
            Vec3{std::min(origin.x, end.x) - padding,
                 std::min(origin.y, end.y) - padding,
                 std::min(origin.z, end.z) - padding},
            Vec3{std::max(origin.x, end.x) + padding,
                 std::max(origin.y, end.y) + padding,
                 std::max(origin.z, end.z) + padding},
        };

        const double cell_size = std::max(1.0, max_range * 0.25);
        ensure_spatial_index(cell_size);

        auto min_cell = cell(window.min_corner, cell_size);
        auto max_cell = cell(window.max_corner, cell_size);
        std::vector<std::uint8_t> seen(obstacles_.size(), 0);
        for (int ix = min_cell[0]; ix <= max_cell[0]; ++ix) {
            for (int iy = min_cell[1]; iy <= max_cell[1]; ++iy) {
                for (int iz = min_cell[2]; iz <= max_cell[2]; ++iz) {
                    auto it = spatial_index_.find({ix, iy, iz});
                    if (it == spatial_index_.end()) continue;
                    for (std::size_t obs_idx : it->second) {
                        if (seen[obs_idx]) continue;
                        seen[obs_idx] = 1;
                        if (overlaps(bounds_of(obstacles_[obs_idx]), window)) {
                            out.push_back(&obstacles_[obs_idx]);
                        }
                    }
                }
            }
        }
        return out;
    }

    void remove_at(std::size_t index) {
        if (index >= obstacles_.size()) return;
        obstacles_.erase(obstacles_.begin() + static_cast<std::ptrdiff_t>(index));
        if (index < ids_.size()) ids_.erase(ids_.begin() + static_cast<std::ptrdiff_t>(index));
        invalidate_spatial_index();
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
        invalidate_spatial_index();
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

    void invalidate_spatial_index() const {
        spatial_index_.clear();
        spatial_index_cell_size_ = 0.0;
        spatial_index_count_ = 0;
    }

    static std::array<int, 3> cell(const Vec3& p, double cell_size) {
        return {
            static_cast<int>(std::floor(p.x / cell_size)),
            static_cast<int>(std::floor(p.y / cell_size)),
            static_cast<int>(std::floor(p.z / cell_size)),
        };
    }

    static Bounds bounds_of(const ObstacleVariant& obs) {
        return std::visit([](const auto& o) -> Bounds {
            using T = std::decay_t<decltype(o)>;
            if constexpr (std::is_same_v<T, AABB>) {
                return Bounds{o.min_corner, o.max_corner};
            } else if constexpr (std::is_same_v<T, Sphere>) {
                Vec3 r{o.radius, o.radius, o.radius};
                return Bounds{o.center - r, o.center + r};
            } else {
                Vec3 mn{o.center_xy.x - o.radius, o.center_xy.y - o.radius, o.z_min};
                Vec3 mx{o.center_xy.x + o.radius, o.center_xy.y + o.radius, o.z_max};
                return Bounds{mn, mx};
            }
        }, obs);
    }

    static bool overlaps(const Bounds& a, const Bounds& b) {
        return a.max_corner.x >= b.min_corner.x && a.min_corner.x <= b.max_corner.x
            && a.max_corner.y >= b.min_corner.y && a.min_corner.y <= b.max_corner.y
            && a.max_corner.z >= b.min_corner.z && a.min_corner.z <= b.max_corner.z;
    }

    void ensure_spatial_index(double cell_size) const {
        if (!spatial_index_.empty()
            && spatial_index_cell_size_ == cell_size
            && spatial_index_count_ == obstacles_.size()) {
            return;
        }

        spatial_index_.clear();
        spatial_index_cell_size_ = cell_size;
        spatial_index_count_ = obstacles_.size();
        for (std::size_t obs_idx = 0; obs_idx < obstacles_.size(); ++obs_idx) {
            Bounds b = bounds_of(obstacles_[obs_idx]);
            auto min_cell = cell(b.min_corner, cell_size);
            auto max_cell = cell(b.max_corner, cell_size);
            for (int ix = min_cell[0]; ix <= max_cell[0]; ++ix) {
                for (int iy = min_cell[1]; iy <= max_cell[1]; ++iy) {
                    for (int iz = min_cell[2]; iz <= max_cell[2]; ++iz) {
                        spatial_index_[{ix, iy, iz}].push_back(obs_idx);
                    }
                }
            }
        }
    }

    SDFCallback sdf_cb_ = nullptr;
    const void* sdf_ctx_ = nullptr;
    std::vector<ObstacleVariant> obstacles_;
    std::vector<std::string> ids_;
    mutable std::map<std::array<int, 3>, std::vector<std::size_t>> spatial_index_;
    mutable double spatial_index_cell_size_ = 0.0;
    mutable std::size_t spatial_index_count_ = 0;
    int dyn_counter_ = 0;
};

}  // namespace sim
