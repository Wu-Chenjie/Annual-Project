#include "visibility_graph.hpp"

#include <algorithm>
#include <cmath>
#include <variant>

namespace sim {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

VisibilityGraph::VisibilityGraph(int angular_res, double buffer_zone, int n_sphere)
    : angular_res_(angular_res), buffer_zone_(buffer_zone), n_sphere_(n_sphere) {}

void VisibilityGraph::build(const Vec3& start, const Vec3& goal, const ObstacleField& obstacle_field,
                            double visible_range) {
    build_obstacle_graph(obstacle_field, visible_range);
    set_start_goal(start, goal, obstacle_field, visible_range);
}

void VisibilityGraph::build_obstacle_graph(const ObstacleField& obstacle_field, double visible_range) {
    vertices.clear();
    adjacency.clear();
    vertex_to_obs.clear();

    int obs_idx = 0;
    for (const auto& obs : obstacle_field.obstacles()) {
        auto obs_vertices = extract_obstacle_vertices(obs);
        for (const auto& v : obs_vertices) {
            vertices.push_back(v);
            vertex_to_obs.push_back(obs_idx);
        }
        ++obs_idx;
    }

    adjacency.assign(vertices.size(), {});
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        for (std::size_t j = i + 1; j < vertices.size(); ++j) {
            if (is_visible(vertices[i], vertices[j], obstacle_field, visible_range)) {
                adjacency[i].push_back(static_cast<int>(j));
                adjacency[j].push_back(static_cast<int>(i));
            }
        }
    }
}

void VisibilityGraph::set_start_goal(const Vec3& start, const Vec3& goal,
                                     const ObstacleField& obstacle_field, double visible_range) {
    while (!vertex_to_obs.empty() && (vertex_to_obs[0] == -1 || vertex_to_obs[0] == -2)) {
        vertices.erase(vertices.begin());
        vertex_to_obs.erase(vertex_to_obs.begin());
    }

    vertices.insert(vertices.begin(), start);
    vertex_to_obs.insert(vertex_to_obs.begin(), -1);
    vertices.insert(vertices.begin() + 1, goal);
    vertex_to_obs.insert(vertex_to_obs.begin() + 1, -2);

    adjacency.assign(vertices.size(), {});
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        for (std::size_t j = i + 1; j < vertices.size(); ++j) {
            if (is_visible(vertices[i], vertices[j], obstacle_field, visible_range)) {
                adjacency[i].push_back(static_cast<int>(j));
                adjacency[j].push_back(static_cast<int>(i));
            }
        }
    }
}

bool VisibilityGraph::is_visible(const Vec3& p1, const Vec3& p2,
                                 const ObstacleField& obstacle_field, double visible_range) const {
    Vec3 diff = p2 - p1;
    double dist = norm(diff);
    if (dist < 1e-9 || dist > visible_range) return false;
    int n_samples = std::max(2, static_cast<int>(dist / 0.2));
    for (int k = 0; k <= n_samples; ++k) {
        double t = static_cast<double>(k) / static_cast<double>(n_samples);
        Vec3 pt = p1 + diff * t;
        if (obstacle_field.signed_distance(pt) < 0.0) return false;
    }
    return true;
}

std::vector<Vec3> VisibilityGraph::extract_obstacle_vertices(const ObstacleVariant& obs) const {
    if (std::holds_alternative<Sphere>(obs)) return sphere_vertices(std::get<Sphere>(obs));
    if (std::holds_alternative<Cylinder>(obs)) return cylinder_vertices(std::get<Cylinder>(obs));
    return aabb_vertices(std::get<AABB>(obs));
}

std::vector<Vec3> VisibilityGraph::aabb_vertices(const AABB& obs) const {
    Vec3 c = (obs.min_corner + obs.max_corner) * 0.5;
    Vec3 h = (obs.max_corner - obs.min_corner) * 0.5 + Vec3{buffer_zone_, buffer_zone_, buffer_zone_};
    std::vector<Vec3> out;
    out.reserve(8);
    for (int dx : {-1, 1})
    for (int dy : {-1, 1})
    for (int dz : {-1, 1}) {
        out.push_back(c + Vec3{dx * h.x, dy * h.y, dz * h.z});
    }
    return out;
}

std::vector<Vec3> VisibilityGraph::cylinder_vertices(const Cylinder& obs) const {
    std::vector<Vec3> out;
    double r = obs.radius + buffer_zone_;
    out.reserve(static_cast<std::size_t>(angular_res_ * 2));
    for (double z : {obs.z_min, obs.z_max}) {
        for (int k = 0; k < angular_res_; ++k) {
            double a = 2.0 * kPi * static_cast<double>(k) / static_cast<double>(angular_res_);
            out.push_back({obs.center_xy.x + r * std::cos(a),
                           obs.center_xy.y + r * std::sin(a),
                           z});
        }
    }
    return out;
}

std::vector<Vec3> VisibilityGraph::sphere_vertices(const Sphere& obs) const {
    auto dirs = fibonacci_sphere(n_sphere_);
    std::vector<Vec3> out;
    out.reserve(dirs.size());
    double r = obs.radius + buffer_zone_;
    for (const auto& d : dirs) out.push_back(obs.center + d * r);
    return out;
}

std::vector<Vec3> VisibilityGraph::fibonacci_sphere(int n) const {
    std::vector<Vec3> pts;
    n = std::max(1, n);
    pts.reserve(static_cast<std::size_t>(n));
    if (n == 1) return {{1.0, 0.0, 0.0}};
    double golden = kPi * (3.0 - std::sqrt(5.0));
    for (int i = 0; i < n; ++i) {
        double y = 1.0 - 2.0 * static_cast<double>(i) / static_cast<double>(n - 1);
        double r = std::sqrt(std::max(0.0, 1.0 - y * y));
        double theta = golden * static_cast<double>(i);
        pts.push_back({std::cos(theta) * r, y, std::sin(theta) * r});
    }
    return pts;
}

}  // namespace sim
