#pragma once

#include <string>
#include <vector>

#include "math_utils.hpp"

namespace sim {

struct VoronoiRegionScore {
    bool enabled = false;
    std::string region_id;
    int side = 0;
    double stability_bonus = 0.0;
    int obstacle_count = 0;
};

class VoronoiRegionSelector {
public:
    explicit VoronoiRegionSelector(double weight = 0.25);

    [[nodiscard]] VoronoiRegionScore score(
        const Vec3& pose,
        const Vec3& goal,
        const Vec3& candidate,
        const std::vector<Vec3>& obstacle_centers,
        const std::string& previous_region_id = {},
        int previous_side = 0
    ) const;

private:
    double weight_ = 0.25;
};

}  // namespace sim
