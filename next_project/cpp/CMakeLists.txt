cmake_minimum_required(VERSION 3.20)
project(next_project_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
    src/obstacle_scenario.cpp
    src/formation_simulation.cpp
    src/visualization.cpp
)

add_library(sim_core STATIC ${CORE_SOURCES})
target_include_directories(sim_core PUBLIC include)

add_executable(sim_main src/main.cpp)
target_link_libraries(sim_main PRIVATE sim_core)

add_executable(sim_benchmark src/benchmark.cpp)
target_link_libraries(sim_benchmark PRIVATE sim_core)

add_executable(sim_warehouse src/warehouse_main.cpp)
target_link_libraries(sim_warehouse PRIVATE sim_core)

add_executable(sim_dynamic_replay src/dynamic_main.cpp)
target_link_libraries(sim_dynamic_replay PRIVATE sim_core)
if(MINGW)
    target_link_options(sim_dynamic_replay PRIVATE -static -static-libgcc -static-libstdc++)
endif()
