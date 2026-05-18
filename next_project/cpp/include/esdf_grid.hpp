#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <queue>
#include <vector>

#include "math_utils.hpp"
#include "occupancy_grid.hpp"

namespace sim {

// 三维欧几里得有符号距离场：O(1) 查表替代 O(N) 障碍物遍历
class ESDFGrid {
public:
    ESDFGrid() = default;

    // 从 OccupancyGrid 构建（occupied>=1 的体素作为障碍物源点）
    void build(const OccupancyGrid& grid, double truncation_distance = -1.0) {
        origin_ = grid.origin;
        resolution_ = grid.resolution;
        nx_ = grid.nx;
        ny_ = grid.ny;
        nz_ = grid.nz;
        size_t total = static_cast<size_t>(nx_) * ny_ * nz_;

        outside_.assign(total, INF);
        inside_.assign(total, INF);
        grad_.assign(total, Vec3{});

        // 多源 BFS：所有 occupied cell 作为源点，距离=0
        std::queue<std::array<int, 3>> q;
        for (int iz = 0; iz < nz_; ++iz)
            for (int iy = 0; iy < ny_; ++iy)
                for (int ix = 0; ix < nx_; ++ix) {
                    if (grid.data[idx(ix, iy, iz)] >= 1) {
                        size_t i = idx(ix, iy, iz);
                        outside_[i] = 0.0;
                        q.push({ix, iy, iz});
                    }
                }

        // 26 邻域扩散（outside distance: free→obstacle）
        bfs_expand(q, outside_, truncation_distance);

        // 构建 inside distance（obstacle→free）：从 free cell 反向 BFS
        std::queue<std::array<int, 3>> q_in;
        for (int iz = 0; iz < nz_; ++iz)
            for (int iy = 0; iy < ny_; ++iy)
                for (int ix = 0; ix < nx_; ++ix) {
                    if (grid.data[idx(ix, iy, iz)] == 0) {
                        size_t i = idx(ix, iy, iz);
                        inside_[i] = 0.0;
                        q_in.push({ix, iy, iz});
                    }
                }
        bfs_expand(q_in, inside_, truncation_distance);

        // 预计算梯度
        build_gradient();
    }

    // O(1) 有符号距离查询
    [[nodiscard]] double signed_distance(const Vec3& p) const {
        auto [ix, iy, iz, fx, fy, fz] = world_to_frac(p);
        if (!inside_bounds(ix, iy, iz)) {
            // 在网格外：返回最近边界距离 + 到边界外点的距离
            int cx = std::clamp(ix, 0, nx_ - 1);
            int cy = std::clamp(iy, 0, ny_ - 1);
            int cz = std::clamp(iz, 0, nz_ - 1);
            double d_boundary = trilinear_sample(outside_, cx, cy, cz, fx, fy, fz);
            Vec3 clamped{std::clamp(p.x, origin_.x, origin_.x + (nx_ - 1) * resolution_),
                         std::clamp(p.y, origin_.y, origin_.y + (ny_ - 1) * resolution_),
                         std::clamp(p.z, origin_.z, origin_.z + (nz_ - 1) * resolution_)};
            return d_boundary + norm(p - clamped);
        }

        double d_out = trilinear_sample(outside_, ix, iy, iz, fx, fy, fz);
        double d_in = trilinear_sample(inside_, ix, iy, iz, fx, fy, fz);

        // occupied → negative signed distance
        if (d_out < 1e-9) return -d_in;
        return d_out;
    }

    // O(1) 梯度查询
    [[nodiscard]] Vec3 gradient(const Vec3& p) const {
        auto [ix, iy, iz, fx, fy, fz] = world_to_frac(p);
        if (!inside_bounds(ix, iy, iz)) {
            Vec3 grad{};
            return grad;
        }
        return trilinear_sample_vec(grad_, ix, iy, iz, fx, fy, fz);
    }

    // O(1) 占据查询
    [[nodiscard]] bool is_occupied(const Vec3& p) const {
        return signed_distance(p) <= 0.0;
    }

    [[nodiscard]] bool is_occupied_world(const Vec3& p) const {
        auto [ix, iy, iz, fx, fy, fz] = world_to_frac(p);
        if (!inside_bounds(ix, iy, iz)) return false;
        return outside_[idx(ix, iy, iz)] < 1e-9;
    }

    [[nodiscard]] Vec3 origin() const { return origin_; }
    [[nodiscard]] double resolution() const { return resolution_; }

private:
    static constexpr double INF = 1e18;
    Vec3 origin_{};
    double resolution_ = 1.0;
    int nx_ = 0, ny_ = 0, nz_ = 0;

    std::vector<float> outside_;  // free space → nearest obstacle
    std::vector<float> inside_;   // obstacle → nearest free space
    std::vector<Vec3> grad_;

    [[nodiscard]] size_t idx(int x, int y, int z) const {
        return (static_cast<size_t>(z) * ny_ + y) * nx_ + x;
    }

    [[nodiscard]] bool inside_bounds(int x, int y, int z) const {
        return x >= 0 && x < nx_ && y >= 0 && y < ny_ && z >= 0 && z < nz_;
    }

    [[nodiscard]] Vec3 cell_world(int ix, int iy, int iz) const {
        return Vec3{origin_.x + ix * resolution_,
                    origin_.y + iy * resolution_,
                    origin_.z + iz * resolution_};
    }

    // 返回 (ix, iy, iz, frac_x, frac_y, frac_z)
    struct FracResult { int ix, iy, iz; double fx, fy, fz; };
    [[nodiscard]] FracResult world_to_frac(const Vec3& p) const {
        double gx = (p.x - origin_.x) / resolution_;
        double gy = (p.y - origin_.y) / resolution_;
        double gz = (p.z - origin_.z) / resolution_;
        int ix = static_cast<int>(std::floor(gx));
        int iy = static_cast<int>(std::floor(gy));
        int iz = static_cast<int>(std::floor(gz));
        return {ix, iy, iz, gx - ix, gy - iy, gz - iz};
    }

    // 26 邻域 BFS 扩散
    void bfs_expand(std::queue<std::array<int, 3>>& q, std::vector<float>& dist, double trunc) {
        // 预计算 26 邻域偏移及步长（一次，非 static 避免成员捕获问题）
        struct Nb { int dx, dy, dz; double step; };
        std::array<Nb, 26> nb;
        int ni = 0;
        for (int dz = -1; dz <= 1; ++dz)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    int ad = std::abs(dx) + std::abs(dy) + std::abs(dz);
                    double step = (ad == 1) ? 1.0 : (ad == 2 ? std::sqrt(2.0) : std::sqrt(3.0));
                    nb[ni++] = {dx, dy, dz, step * resolution_};
                }

        while (!q.empty()) {
            auto [x, y, z] = q.front();
            q.pop();
            size_t ci = idx(x, y, z);
            double cd = dist[ci];

            if (trunc > 0 && cd >= trunc) continue;

            for (const auto& n : nb) {
                int nx = x + n.dx, ny = y + n.dy, nz = z + n.dz;
                if (!inside_bounds(nx, ny, nz)) continue;
                size_t nidx = idx(nx, ny, nz);
                double nd = cd + n.step;
                if (nd < dist[nidx]) {
                    dist[nidx] = static_cast<float>(nd);
                    q.push({nx, ny, nz});
                }
            }
        }
    }

    // 三线性插值
    [[nodiscard]] double trilinear_sample(const std::vector<float>& data,
                                           int ix, int iy, int iz,
                                           double fx, double fy, double fz) const {
        ix = std::clamp(ix, 0, nx_ - 1);
        iy = std::clamp(iy, 0, ny_ - 1);
        iz = std::clamp(iz, 0, nz_ - 1);
        int ix1 = std::min(ix + 1, nx_ - 1);
        int iy1 = std::min(iy + 1, ny_ - 1);
        int iz1 = std::min(iz + 1, nz_ - 1);

        double c000 = data[idx(ix,  iy,  iz)];
        double c100 = data[idx(ix1, iy,  iz)];
        double c010 = data[idx(ix,  iy1, iz)];
        double c110 = data[idx(ix1, iy1, iz)];
        double c001 = data[idx(ix,  iy,  iz1)];
        double c101 = data[idx(ix1, iy,  iz1)];
        double c011 = data[idx(ix,  iy1, iz1)];
        double c111 = data[idx(ix1, iy1, iz1)];

        double c00 = c000 + (c100 - c000) * fx;
        double c01 = c001 + (c101 - c001) * fx;
        double c10 = c010 + (c110 - c010) * fx;
        double c11 = c011 + (c111 - c011) * fx;
        double c0 = c00 + (c10 - c00) * fy;
        double c1 = c01 + (c11 - c01) * fy;
        return c0 + (c1 - c0) * fz;
    }

    Vec3 trilinear_sample_vec(const std::vector<Vec3>& data,
                               int ix, int iy, int iz,
                               double fx, double fy, double fz) const {
        ix = std::clamp(ix, 0, nx_ - 1);
        iy = std::clamp(iy, 0, ny_ - 1);
        iz = std::clamp(iz, 0, nz_ - 1);
        int ix1 = std::min(ix + 1, nx_ - 1);
        int iy1 = std::min(iy + 1, ny_ - 1);
        int iz1 = std::min(iz + 1, nz_ - 1);

        Vec3 c000 = data[idx(ix,  iy,  iz)];
        Vec3 c100 = data[idx(ix1, iy,  iz)];
        Vec3 c010 = data[idx(ix,  iy1, iz)];
        Vec3 c110 = data[idx(ix1, iy1, iz)];
        Vec3 c001 = data[idx(ix,  iy,  iz1)];
        Vec3 c101 = data[idx(ix1, iy,  iz1)];
        Vec3 c011 = data[idx(ix,  iy1, iz1)];
        Vec3 c111 = data[idx(ix1, iy1, iz1)];

        Vec3 c00 = c000 + (c100 - c000) * fx;
        Vec3 c01 = c001 + (c101 - c001) * fx;
        Vec3 c10 = c010 + (c110 - c010) * fx;
        Vec3 c11 = c011 + (c111 - c011) * fx;
        Vec3 c0 = c00 + (c10 - c00) * fy;
        Vec3 c1 = c01 + (c11 - c01) * fy;
        return c0 + (c1 - c0) * fz;
    }

    // 中心差分梯度
    void build_gradient() {
        grad_.resize(outside_.size(), Vec3{});
        double step = resolution_ * 2.0;
        for (int iz = 1; iz < nz_ - 1; ++iz)
            for (int iy = 1; iy < ny_ - 1; ++iy)
                for (int ix = 1; ix < nx_ - 1; ++ix) {
                    size_t i = idx(ix, iy, iz);
                    // 用 outside 距离场计算梯度
                    double dx = (outside_[idx(ix+1, iy, iz)] - outside_[idx(ix-1, iy, iz)]) / step;
                    double dy = (outside_[idx(ix, iy+1, iz)] - outside_[idx(ix, iy-1, iz)]) / step;
                    double dz = (outside_[idx(ix, iy, iz+1)] - outside_[idx(ix, iy, iz-1)]) / step;
                    double len = std::sqrt(dx*dx + dy*dy + dz*dz);
                    if (len > 1e-12) {
                        grad_[i] = Vec3{dx/len, dy/len, dz/len};
                    }
                }
    }
};

}  // namespace sim
