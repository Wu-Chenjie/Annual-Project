#pragma once

#include <string>
#include <utility>

#include "math_utils.hpp"
#include "obstacles.hpp"

namespace sim {

std::pair<ObstacleField, std::array<Vec3, 2>> load_from_json(const std::string& filepath);

}  // namespace sim
