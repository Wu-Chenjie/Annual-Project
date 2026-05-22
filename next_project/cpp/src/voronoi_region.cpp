#include "voronoi_region.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace sim {

VoronoiRegionSelector::VoronoiRegionSelector(double weight)
    : weight_(std::max(0.0, weight)) {}

VoronoiRegionScore VoronoiRegionSelector::score(
    const Vec3& pose,
    const Vec3& goal,
    const Vec3& candidate,
    const std::vector<Vec3>& obstacle_centers,
    const std::string& previous_region_id,
    int previous_side
) const {
    VoronoiRegionScore out;
    out.obstacle_count = static_cast<int>(obstacle_centers.size());
    if (obstacle_centers.size() < 2) {
        return out;
    }

    const Vec3 axis{goal.x - pose.x, goal.y - pose.y, 0.0};
    const Vec3 rel{candidate.x - pose.x, candidate.y - pose.y, 0.0};
    const double axis_norm = std::sqrt(axis.x * axis.x + axis.y * axis.y);

    std::vector<double> distances(obstacle_centers.size(), 0.0);
    if (axis_norm <= 1e-9) {
        out.side = previous_side > 0 ? 1 : previous_side < 0 ? -1 : 0;
        for (std::size_t i = 0; i < obstacle_centers.size(); ++i) {
            const double dx = obstacle_centers[i].x - candidate.x;
            const double dy = obstacle_centers[i].y - candidate.y;
            distances[i] = std::sqrt(dx * dx + dy * dy);
        }
    } else {
        const Vec3 axis_unit{axis.x / axis_norm, axis.y / axis_norm, 0.0};
        const double cand_s = rel.x * axis_unit.x + rel.y * axis_unit.y;
        for (std::size_t i = 0; i < obstacle_centers.size(); ++i) {
            const double center_s = (obstacle_centers[i].x - pose.x) * axis_unit.x
                                  + (obstacle_centers[i].y - pose.y) * axis_unit.y;
            distances[i] = std::abs(center_s - cand_s);
        }
        const double cross_z = axis.x * rel.y - axis.y * rel.x;
        out.side = cross_z > 1e-9 ? 1 : cross_z < -1e-9 ? -1 : 0;
    }

    std::vector<std::size_t> order(obstacle_centers.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::partial_sort(order.begin(), order.begin() + 2, order.end(),
                      [&](std::size_t lhs, std::size_t rhs) {
                          if (distances[lhs] != distances[rhs])
                              return distances[lhs] < distances[rhs];
                          return lhs < rhs;  // deterministic tie-breaker
                      });
    const int first = static_cast<int>(std::min(order[0], order[1]));
    const int second = static_cast<int>(std::max(order[0], order[1]));
    out.region_id = std::to_string(first) + ":" + std::to_string(second);
    out.enabled = true;

    if (!previous_region_id.empty() && previous_region_id == out.region_id
        && previous_side != 0 && out.side != 0) {
        out.stability_bonus = out.side == previous_side ? weight_ : -weight_;
    } else if (previous_side != 0 && out.side != 0) {
        out.stability_bonus = out.side == previous_side ? 0.5 * weight_ : -0.5 * weight_;
    }

    return out;
}

}  // namespace sim
