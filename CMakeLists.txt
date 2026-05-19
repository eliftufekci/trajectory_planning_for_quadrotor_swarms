cmake_minimum_required(VERSION 3.10)
project(QuadrotorSwarmPlanner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON) 
set(CMAKE_BUILD_TYPE Debug)

# Generate compile_commands.json for IDEs and clangd.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies must be installed on the system.
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system filesystem)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_package(osqp REQUIRED)

# Enable osqp-eigen debug output.
add_compile_definitions(OSQP_EIGEN_DEBUG_OUTPUT)

# Build osqp-eigen as a subproject and expose OsqpEigen::OsqpEigen.
add_subdirectory(libs/osqp-eigen)

file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

add_executable(env_test
    ${SOURCE_FILES}
)

target_include_directories(env_test PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${OMPL_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/libs/libMultiRobotPlanning/include
)

target_link_libraries(env_test
    ${OCTOMAP_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${FCL_LIBRARIES}
    yaml-cpp
    Eigen3::Eigen
    osqp::osqp
    OsqpEigen::OsqpEigen
)

message(STATUS "Eigen3 : ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "OctoMap : ${OCTOMAP_LIBRARIES}")
message(STATUS "OMPL : ${OMPL_INCLUDE_DIRS}")
message(STATUS "FCL : ${FCL_LIBRARIES}")
message(STATUS "OSQP : ${OSQP_LIBRARIES}")
