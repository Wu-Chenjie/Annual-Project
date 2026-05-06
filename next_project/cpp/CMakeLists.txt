cmake_minimum_required(VERSION 3.20)
project(next_project_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

if(MSVC)
    add_compile_options(/O2 /W4)
else()
    add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

include_directories(include)

set(CORE_SOURCES
    src/rotor.cpp
    src/allocator.cpp
    src/wind_field.cpp
    src/smc.cpp
    src/controller.cpp
    src/topology.cpp
    src/drone.cpp
    src/occupancy_grid.cpp
    src/map_loader.cpp
    src/firi.cpp
    src/visibility_graph.cpp
    src/gnn_planner.cpp
    src/artificial_potential_field.cpp
    src/formation_apf.cpp
    src/obstacle_scenario.cpp
    src/formation_simulation.cpp
    src/visualization.cpp
)

add_library(sim_core STATIC ${CORE_SOURCES})
target_include_directories(sim_core PUBLIC include)
set_target_properties(sim_core PROPERTIES CXX_SCAN_FOR_MODULES OFF)

add_executable(sim_main src/main.cpp)
target_link_libraries(sim_main PRIVATE sim_core)
set_target_properties(sim_main PROPERTIES CXX_SCAN_FOR_MODULES OFF)

add_executable(sim_benchmark src/benchmark.cpp)
target_link_libraries(sim_benchmark PRIVATE sim_core)
set_target_properties(sim_benchmark PROPERTIES CXX_SCAN_FOR_MODULES OFF)

add_executable(sim_warehouse src/warehouse_main.cpp)
target_link_libraries(sim_warehouse PRIVATE sim_core)
set_target_properties(sim_warehouse PROPERTIES CXX_SCAN_FOR_MODULES OFF)

add_executable(sim_dynamic_replay src/dynamic_main.cpp)
target_link_libraries(sim_dynamic_replay PRIVATE sim_core)
set_target_properties(sim_dynamic_replay PROPERTIES CXX_SCAN_FOR_MODULES OFF)

add_executable(sim_apf_formation_probe src/apf_formation_probe.cpp)
target_link_libraries(sim_apf_formation_probe PRIVATE sim_core)
set_target_properties(sim_apf_formation_probe PROPERTIES CXX_SCAN_FOR_MODULES OFF)
