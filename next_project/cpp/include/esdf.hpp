#pragma once

#include <cmath>
#include <deque>
#include <limits>
#include <vector>

#include "math_utils.hpp"
#include "occupancy_grid.hpp"

namespace sim {

inline std::vector<double> compute_esdf(const OccupancyGrid& grid, double max_dist = -1.0) {
    int nx = grid.nx, ny = grid.ny, nz = grid.nz;
    std::vector<double> dist(nx * ny * nz, std::numeric_limits<double>::infinity());
    std::deque<std::array<int, 3>> q;
    double max_vox = (max_dist > 0) ? max_dist : std::max({nx, ny, nz}) * grid.resolution;

    for (int iz = 0; iz < nz; ++iz)
        for (int iy = 0; iy < ny; ++iy)
            for (int ix = 0; ix < nx; ++ix) {
                if (grid.is_occupied(ix, iy, iz)) {
                    dist[(iz * ny + iy) * nx + ix] = 0.0;
                    q.push_back({ix, iy, iz});
                }
            }

    const int dirs[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    while (!q.empty()) {
        auto [cx, cy, cz] = q.front(); q.pop_front();
        double d = dist[(cz * ny + cy) * nx + cx];
        if (d >= max_vox) continue;
        for (const auto& dr : dirs) {
            int nx_i = cx + dr[0], ny_i = cy + dr[1], nz_i = cz + dr[2];
            if (nx_i < 0 || nx_i >= nx || ny_i < 0 || ny_i >= ny || nz_i < 0 || nz_i >= nz) continue;
            double nd = d + grid.resolution;
            auto& cell = dist[(nz_i * ny + ny_i) * nx + nx_i];
            if (nd < cell) {
                cell = nd;
                q.push_back({nx_i, ny_i, nz_i});
            }
        }
    }

    for (auto& v : dist) if (std::isinf(v)) v = max_vox;
    return dist;
}

class CostAwareGrid : public OccupancyGrid {
public:
    CostAwareGrid(const OccupancyGrid& base, const std::vector<double>& esdf,
                  double weight = 2.0, double scale = 1.5, double cap_distance = 4.0)
        : OccupancyGrid(base), esdf_(esdf), weight_(weight),
          scale_(scale), cap_distance_(cap_distance) {}

    [[nodiscard]] double extra_cost(int ix, int iy, int iz) const {
        if (ix < 0 || ix >= nx || iy < 0 || iy >= ny || iz < 0 || iz >= nz) return weight_;
        double d = esdf_[(iz * ny + iy) * nx + ix];
        if (d >= cap_distance_) return 0.0;
        return weight_ * std::exp(-d / scale_);
    }

private:
    std::vector<double> esdf_;
    double weight_, scale_, cap_distance_;
};

}  // namespace sim
