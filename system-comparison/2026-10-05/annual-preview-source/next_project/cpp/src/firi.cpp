#include "firi.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <variant>

namespace sim {

bool FIRICorridor::contains(const Vec3& point, double tol) const {
    for (std::size_t i = 0; i < normals.size(); ++i) {
        if (dot(normals[i], point) > offsets[i] + tol) return false;
    }
    return true;
}

Vec3 FIRICorridor::project(Vec3 point, int max_iter) const {
    for (int iter = 0; iter < max_iter; ++iter) {
        bool changed = false;
        for (std::size_t i = 0; i < normals.size(); ++i) {
            const Vec3& normal = normals[i];
            double violation = dot(normal, point) - offsets[i];
            if (violation <= 0.0) continue;
            double denom = dot(normal, normal);
            if (denom <= 1e-12) continue;
            point = point - normal * (violation / denom);
            changed = true;
        }
        if (!changed) break;
    }
    return point;
}

FIRIRefiner::FIRIRefiner(const ObstacleField& obstacle_field, double min_clearance,
                         double sample_step, int max_projection_iter)
    : obstacle_field_(&obstacle_field),
      min_clearance_(min_clearance),
      sample_step_(sample_step),
      max_projection_iter_(max_projection_iter) {}

std::vector<Vec3> FIRIRefiner::refine(const std::vector<Vec3>& path,
                                      const std::vector<Vec3>& seeds_in) const {
    if (path.size() < 2) return path;

    const std::vector<Vec3>& raw_seeds = seeds_in.size() >= 2 ? seeds_in : path;
    auto seeds = prepare_seeds(raw_seeds);
    if (seeds.size() < 2) return remove_duplicate_points(push_path_out_if_needed(path));

    std::vector<Vec3> refined;
    refined.push_back(seeds.front());
    for (std::size_t i = 0; i + 1 < seeds.size(); ++i) {
        auto corridor = build_segment_corridor(seeds[i], seeds[i + 1]);
        auto segment = refine_segment(corridor);
        if (segment.size() <= 1) continue;
        segment.front() = seeds[i];
        segment.back() = seeds[i + 1];
        refined.insert(refined.end(), segment.begin() + 1, segment.end());
    }

    refined = push_path_out_if_needed(refined);
    if (!refined.empty()) {
        refined.front() = seeds.front();
        refined.back() = seeds.back();
    }
    return remove_duplicate_points(refined);
}

std::vector<Vec3> FIRIRefiner::prepare_seeds(const std::vector<Vec3>& seeds) const {
    auto out = seeds;
    if (out.size() <= 2) return push_path_out_if_needed(out);
    for (std::size_t i = 1; i + 1 < out.size(); ++i) {
        if (obstacle_field_->signed_distance(out[i]) < min_clearance_) {
            out[i] = push_out_if_needed(out[i]);
        }
    }
    return out;
}

FIRICorridor FIRIRefiner::build_segment_corridor(const Vec3& start, const Vec3& goal) const {
    FIRICorridor corridor;
    corridor.start = start;
    corridor.goal = goal;
    corridor.min_clearance = min_clearance_;

    auto seed_points = sample_segment(start, goal, sample_step_);
    Vec3 seed_min = seed_points.front();
    Vec3 seed_max = seed_points.front();
    for (const auto& p : seed_points) {
        seed_min.x = std::min(seed_min.x, p.x);
        seed_min.y = std::min(seed_min.y, p.y);
        seed_min.z = std::min(seed_min.z, p.z);
        seed_max.x = std::max(seed_max.x, p.x);
        seed_max.y = std::max(seed_max.y, p.y);
        seed_max.z = std::max(seed_max.z, p.z);
    }
    seed_min = seed_min - Vec3{min_clearance_, min_clearance_, min_clearance_};
    seed_max = seed_max + Vec3{min_clearance_, min_clearance_, min_clearance_};

    corridor.normals.push_back({1, 0, 0}); corridor.offsets.push_back(seed_max.x);
    corridor.normals.push_back({-1, 0, 0}); corridor.offsets.push_back(-seed_min.x);
    corridor.normals.push_back({0, 1, 0}); corridor.offsets.push_back(seed_max.y);
    corridor.normals.push_back({0, -1, 0}); corridor.offsets.push_back(-seed_min.y);
    corridor.normals.push_back({0, 0, 1}); corridor.offsets.push_back(seed_max.z);
    corridor.normals.push_back({0, 0, -1}); corridor.offsets.push_back(-seed_min.z);

    for (const auto& obs : obstacle_field_->obstacles()) {
        Vec3 normal{};
        double offset = 0.0;
        if (analytic_separating_plane(obs, start, goal, seed_points, normal, offset)) {
            corridor.normals.push_back(normal);
            corridor.offsets.push_back(offset);
        }
    }

    compute_inscribed_ball(corridor);
    return corridor;
}

std::vector<Vec3> FIRIRefiner::refine_segment(const FIRICorridor& corridor) const {
    auto points = sample_segment(corridor.start, corridor.goal, sample_step_);
    if (points.size() < 2) return points;

    std::vector<Vec3> refined;
    refined.push_back(corridor.start);
    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
        Vec3 candidate = corridor.project(points[i], max_projection_iter_);
        candidate = push_out_if_needed(candidate);
        candidate = corridor.project(candidate, max_projection_iter_);
        refined.push_back(candidate);
    }
    refined.push_back(corridor.goal);
    return refined;
}

void FIRIRefiner::compute_inscribed_ball(FIRICorridor& corridor) const {
    Vec3 center = (corridor.start + corridor.goal) * 0.5;
    center = corridor.project(center, max_projection_iter_ * 2);
    double radius = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < corridor.normals.size(); ++i) {
        double nn = norm(corridor.normals[i]);
        if (nn <= 1e-12) continue;
        radius = std::min(radius, (corridor.offsets[i] - dot(corridor.normals[i], center)) / nn);
    }
    if (!std::isfinite(radius)) radius = min_clearance_;
    corridor.ellipsoid_center = center;
    corridor.inscribed_radius = std::max(radius, 1e-6);
}

std::vector<Vec3> FIRIRefiner::sample_segment(const Vec3& start, const Vec3& goal, double step) const {
    double dist = norm(goal - start);
    if (dist < 1e-12) return {start};
    int n = std::max(2, static_cast<int>(std::ceil(dist / std::max(step, 1e-6))) + 1);
    std::vector<Vec3> out;
    out.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(n - 1);
        out.push_back(start + (goal - start) * t);
    }
    return out;
}

Vec3 FIRIRefiner::closest_point_on_segment(const Vec3& a, const Vec3& b, const Vec3& p) const {
    Vec3 ab = b - a;
    double denom = dot(ab, ab);
    if (denom < 1e-12) return a;
    double t = dot(p - a, ab) / denom;
    t = clamp(t, 0.0, 1.0);
    return a + ab * t;
}

Vec3 FIRIRefiner::push_out_if_needed(Vec3 point) const {
    if (obstacle_field_->signed_distance(point) >= min_clearance_) return point;

    double eps = 0.05;
    for (int iter = 0; iter < max_projection_iter_ * 2; ++iter) {
        double sd = obstacle_field_->signed_distance(point);
        if (sd >= min_clearance_) return point;
        Vec3 grad{
            obstacle_field_->signed_distance({point.x + eps, point.y, point.z})
                - obstacle_field_->signed_distance({point.x - eps, point.y, point.z}),
            obstacle_field_->signed_distance({point.x, point.y + eps, point.z})
                - obstacle_field_->signed_distance({point.x, point.y - eps, point.z}),
            obstacle_field_->signed_distance({point.x, point.y, point.z + eps})
                - obstacle_field_->signed_distance({point.x, point.y, point.z - eps}),
        };
        grad = grad / (2.0 * eps);
        double gn = norm(grad);
        if (gn < 1e-12) break;
        point = point + (grad / gn) * std::max(0.05, 0.2 * min_clearance_);
    }
    return point;
}

std::vector<Vec3> FIRIRefiner::push_path_out_if_needed(const std::vector<Vec3>& path) const {
    auto out = path;
    for (auto& p : out) p = push_out_if_needed(p);
    return out;
}

std::vector<Vec3> FIRIRefiner::remove_duplicate_points(const std::vector<Vec3>& path) const {
    if (path.size() <= 1) return path;
    std::vector<Vec3> out;
    out.push_back(path.front());
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (norm(path[i] - out.back()) > 1e-9) out.push_back(path[i]);
    }
    return out;
}

bool FIRIRefiner::analytic_separating_plane(const ObstacleVariant& obs, const Vec3& start,
                                            const Vec3& goal, const std::vector<Vec3>& seed_points,
                                            Vec3& normal, double& offset) const {
    Vec3 obs_point = nearest_obstacle_point(obs, start, goal);
    Vec3 seed = closest_point_on_segment(start, goal, obs_point);
    normal = seed - obs_point;
    double nn = norm(normal);
    if (nn < 1e-9) {
        seed = push_out_if_needed(seed);
        normal = seed - obs_point;
        nn = norm(normal);
        if (nn < 1e-9) return false;
    }

    normal = -normal / nn;
    offset = dot(normal, obs_point - normal * min_clearance_);
    for (const auto& p : seed_points) {
        if (dot(normal, p) > offset + 1e-8) return false;
    }
    return true;
}

Vec3 FIRIRefiner::nearest_obstacle_point(const ObstacleVariant& obs, const Vec3& start,
                                         const Vec3& goal) const {
    if (std::holds_alternative<Sphere>(obs)) {
        return nearest_sphere_point(std::get<Sphere>(obs), start, goal);
    }
    if (std::holds_alternative<Cylinder>(obs)) {
        return nearest_cylinder_point(std::get<Cylinder>(obs), start, goal);
    }
    return nearest_aabb_point(std::get<AABB>(obs), start, goal);
}

Vec3 FIRIRefiner::nearest_sphere_point(const Sphere& obs, const Vec3& start, const Vec3& goal) const {
    Vec3 seed = closest_point_on_segment(start, goal, obs.center);
    Vec3 direction = seed - obs.center;
    double dn = norm(direction);
    if (dn < 1e-9) direction = safe_segment_normal(start, goal);
    else direction = direction / dn;
    return obs.center + direction * obs.radius;
}

Vec3 FIRIRefiner::nearest_cylinder_point(const Cylinder& obs, const Vec3& start, const Vec3& goal) const {
    auto samples = sample_segment(start, goal, std::max(sample_step_, 0.2));
    Vec3 best_surface = samples.front();
    double best_dist = std::numeric_limits<double>::infinity();
    for (const auto& seed : samples) {
        Vec3 radial{seed.x - obs.center_xy.x, seed.y - obs.center_xy.y, 0.0};
        double rn = std::sqrt(radial.x * radial.x + radial.y * radial.y);
        Vec3 dir = rn < 1e-9 ? safe_segment_normal(start, goal) : Vec3{radial.x / rn, radial.y / rn, 0.0};
        double dir_norm = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        if (dir_norm < 1e-9) dir = {1.0, 0.0, 0.0};
        else dir = {dir.x / dir_norm, dir.y / dir_norm, 0.0};
        Vec3 surface{
            obs.center_xy.x + obs.radius * dir.x,
            obs.center_xy.y + obs.radius * dir.y,
            clamp(seed.z, obs.z_min, obs.z_max),
        };
        double d = norm(seed - surface);
        if (d < best_dist) {
            best_dist = d;
            best_surface = surface;
        }
    }
    return best_surface;
}

Vec3 FIRIRefiner::nearest_aabb_point(const AABB& obs, const Vec3& start, const Vec3& goal) const {
    auto samples = sample_segment(start, goal, std::max(sample_step_, 0.2));
    Vec3 best_surface = samples.front();
    double best_dist = std::numeric_limits<double>::infinity();
    Vec3 center = (obs.min_corner + obs.max_corner) * 0.5;
    Vec3 half = (obs.max_corner - obs.min_corner) * 0.5;

    for (const auto& seed : samples) {
        Vec3 clipped{
            clamp(seed.x, obs.min_corner.x, obs.max_corner.x),
            clamp(seed.y, obs.min_corner.y, obs.max_corner.y),
            clamp(seed.z, obs.min_corner.z, obs.max_corner.z),
        };
        bool inside = seed.x >= obs.min_corner.x && seed.x <= obs.max_corner.x
                   && seed.y >= obs.min_corner.y && seed.y <= obs.max_corner.y
                   && seed.z >= obs.min_corner.z && seed.z <= obs.max_corner.z;
        if (inside) {
            Vec3 q{std::abs(seed.x - center.x) - half.x,
                   std::abs(seed.y - center.y) - half.y,
                   std::abs(seed.z - center.z) - half.z};
            if (q.x >= q.y && q.x >= q.z) clipped.x = seed.x >= center.x ? obs.max_corner.x : obs.min_corner.x;
            else if (q.y >= q.x && q.y >= q.z) clipped.y = seed.y >= center.y ? obs.max_corner.y : obs.min_corner.y;
            else clipped.z = seed.z >= center.z ? obs.max_corner.z : obs.min_corner.z;
        }
        double d = norm(seed - clipped);
        if (d < best_dist) {
            best_dist = d;
            best_surface = clipped;
        }
    }
    return best_surface;
}

Vec3 FIRIRefiner::safe_segment_normal(const Vec3& start, const Vec3& goal) const {
    Vec3 direction = goal - start;
    double dn = norm(direction);
    if (dn < 1e-9) return {1.0, 0.0, 0.0};
    direction = direction / dn;
    Vec3 ref{0.0, 0.0, 1.0};
    if (std::abs(dot(direction, ref)) > 0.9) ref = {0.0, 1.0, 0.0};
    Vec3 normal = cross(direction, ref);
    double nn = norm(normal);
    if (nn < 1e-9) return {1.0, 0.0, 0.0};
    return normal / nn;
}

}  // namespace sim
