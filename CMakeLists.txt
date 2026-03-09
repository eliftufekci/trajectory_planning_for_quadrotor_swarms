cmake_minimum_required(VERSION 3.10)
project(QuadrotorSwarmPlanner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# ── Paketleri bul ────────────────────────────────────────────────────
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── Include dizinleri ────────────────────────────────────────────────
include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/include
)

# ── Executable ───────────────────────────────────────────────────────
add_executable(env_test
    src/main.cpp
)

target_link_libraries(env_test
    ${OCTOMAP_LIBRARIES}
    yaml-cpp
)

# ── Build sonrası bilgi ──────────────────────────────────────────────
message(STATUS "Eigen3   : ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "OctoMap  : ${OCTOMAP_INCLUDE_DIRS}")
message(STATUS "yaml-cpp : ${YAML_CPP_INCLUDE_DIR}")