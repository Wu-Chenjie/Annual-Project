#include "occupancy_grid.hpp"

namespace sim {

OccupancyGrid OccupancyGrid::inflate(double radius) const {
    int r_vox = static_cast<int>(std::ceil(radius / resolution));
    if (r_vox < 1) r_vox = 1;

    OccupancyGrid out = *this;
    for (int iz = 0; iz < nz; ++iz) {
        for (int iy = 0; iy < ny; ++iy) {
            for (int ix = 0; ix < nx; ++ix) {
                if (!is_occupied(ix, iy, iz)) continue;
                for (int dz = -r_vox; dz <= r_vox; ++dz) {
                    for (int dy = -r_vox; dy <= r_vox; ++dy) {
                        for (int dx = -r_vox; dx <= r_vox; ++dx) {
                            int jx = ix + dx, jy = iy + dy, jz = iz + dz;
                            if (jx < 0 || jx >= nx || jy < 0 || jy >= ny || jz < 0 || jz >= nz) continue;
                            auto& v = out.data[(jz * ny + jy) * nx + jx];
                            if (v == 0) v = 2;
                        }
                    }
                }
            }
        }
    }
    return out;
}

OccupancyGrid OccupancyGrid::from_obstacles(const ObstacleField& field,
                                              const Vec3& origin, const Vec3& extent, double resolution) {
    OccupancyGrid grid;
    grid.origin = origin;
    grid.resolution = resolution;
    grid.nx = static_cast<int>(std::ceil(extent.x / resolution)) + 1;
    grid.ny = static_cast<int>(std::ceil(extent.y / resolution)) + 1;
    grid.nz = static_cast<int>(std::ceil(extent.z / resolution)) + 1;
    grid.data.assign(grid.nx * grid.ny * grid.nz, 0);

    for (int iz = 0; iz < grid.nz; ++iz) {
        for (int iy = 0; iy < grid.ny; ++iy) {
            for (int ix = 0; ix < grid.nx; ++ix) {
                Vec3 p = grid.index_to_world(ix, iy, iz);
                if (field.is_collision(p)) {
                    grid.data[(iz * grid.ny + iy) * grid.nx + ix] = 1;
                }
            }
        }
    }
    return grid;
}

}  // namespace sim
