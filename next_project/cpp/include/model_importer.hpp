#pragma once

#include <array>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "math_utils.hpp"
#include "obstacles.hpp"

namespace sim {

using Vertex = Vec3;
using Triangle = std::array<int, 3>;

// ---- 格式解析 ----

std::pair<std::vector<Vertex>, std::vector<Triangle>> parse_obj(const std::string& text);
std::pair<std::vector<Vertex>, std::vector<Triangle>> parse_stl(const std::vector<uint8_t>& data);
std::pair<std::vector<Vertex>, std::vector<Triangle>> parse_ply(const std::vector<uint8_t>& data);

// ---- 三角面采样 ----

std::vector<Vertex> sample_triangle(const Vertex& a, const Vertex& b, const Vertex& c, double voxel);

// ---- 主入口 ----

std::pair<ObstacleField, std::array<Vec3, 2>> import_model(
    const std::string& filepath,
    double voxel_size = 0.4,
    double scale = 1.0,
    double padding = 0.5,
    int max_obstacles = 500
);

}  // namespace sim
