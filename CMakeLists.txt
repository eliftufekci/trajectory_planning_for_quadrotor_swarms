cmake_minimum_required(VERSION 3.10)
project(QuadrotorSwarm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Eigen3
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(roadmap src/main.cpp)
target_link_libraries(roadmap Eigen3::Eigen)