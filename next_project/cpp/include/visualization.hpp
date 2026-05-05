#pragma once

#include <map>
#include <string>

#include "formation_simulation.hpp"

namespace sim {

class SimulationVisualizer {
public:
    explicit SimulationVisualizer(std::string output_dir = "outputs");

    std::map<std::string, std::string> plot_all(const SimulationResult& result) const;

private:
    std::string plot_trajectory_svg(const SimulationResult& result) const;
    std::string plot_error_components_svg(const SimulationResult& result) const;
    std::string plot_error_stats_svg(const SimulationResult& result) const;

    std::string output_dir_;
};

}  // namespace sim
