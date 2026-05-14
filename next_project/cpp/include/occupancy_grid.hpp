#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "math_utils.hpp"
#include "obstacles.hpp"

namespace sim {

struct OccupancyGrid {
    virtual ~OccupancyGrid() = default;

    Vec3 origin{0, 0, 0};
    double resolution = 0.5;
    int nx = 0, ny = 0, nz = 0;
    std::vector<std::uint8_t> data;  // (nx*ny*nz), 0=free, 1=occupied

    [[nodiscard]] virtual double extra_cost(int, int, int) const { return 0.0; }

    [[nodiscard]] std::array<int, 3> world_to_index(const Vec3& p) const {
        // floor 截断，与 Python astype(int) 行为一致
        int ix = static_cast<int>(std::floor((p.x - origin.x) / resolution + 1e-9));
        int iy = static_cast<int>(std::floor((p.y - origin.y) / resolution + 1e-9));
        int iz = static_cast<int>(std::floor((p.z - origin.z) / resolution + 1e-9));
        return {clamp_int(ix, 0, nx - 1), clamp_int(iy, 0, ny - 1), clamp_int(iz, 0, nz - 1)};
    }

    [[nodiscard]] Vec3 index_to_world(int ix, int iy, int iz) const {
        return Vec3{origin.x + ix * resolution, origin.y + iy * resolution, origin.z + iz * resolution};
    }

    [[nodiscard]] virtual bool is_occupied(int ix, int iy, int iz) const {
        if (ix < 0 || ix >= nx || iy < 0 || iy >= ny || iz < 0 || iz >= nz) return true;
        return data[(iz * ny + iy) * nx + ix] >= 1;
    }

    [[nodiscard]] virtual bool is_free(int ix, int iy, int iz) const {
        return !is_occupied(ix, iy, iz);
    }

    OccupancyGrid inflate(double radius) const;
    OccupancyGrid inflate(const std::array<double, 3>& radius_xyz) const;
    static OccupancyGrid from_obstacles(const ObstacleField& field,
                                         const Vec3& origin, const Vec3& extent, double resolution);

private:
    static int clamp_int(int v, int lo, int hi) {
        return std::max(lo, std::min(v, hi));
    }
};

class SDFAwareGrid : public OccupancyGrid {
public:
    /// NOTE: OccupancyGrid(base) deep-copies the data vector (C++ value semantics).
    /// In Python, SDFAwareGrid shares the same ndarray.  This copy is safe because
    /// SDFAwareGrid is only created in known-map mode where the base grid never
    /// changes after construction.  If unknown-mode ever enables SDF, this class
    /// must be refactored to hold a pointer to the base grid.
    SDFAwareGrid(const OccupancyGrid& base, const ObstacleField& obs, double clearance)
        : OccupancyGrid(base), obs_(&obs), clearance_(clearance) {}

    [[nodiscard]] bool is_occupied(int ix, int iy, int iz) const override {
        if (OccupancyGrid::is_occupied(ix, iy, iz)) return true;
        Vec3 w = index_to_world(ix, iy, iz);
        return obs_->signed_distance(w) < clearance_;
    }
    [[nodiscard]] bool is_free(int ix, int iy, int iz) const override {
        return !is_occupied(ix, iy, iz);
    }

private:
    const ObstacleField* obs_;
    double clearance_;
};

}  // namespace sim
