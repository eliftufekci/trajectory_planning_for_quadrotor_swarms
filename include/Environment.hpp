#pragma once
#include <vector>
#include <string>
#include <stdexcept>
#include <octomap/octomap.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>


// ------------------------------------------------------------
//  Helper: convert a YAML sequence to Vector3d.
// ------------------------------------------------------------
inline Eigen::Vector3d yamlToVec3(const YAML::Node& node);

// ------------------------------------------------------------
//  AABB - Axis-Aligned Bounding Box
// ------------------------------------------------------------
struct AABB {
    Eigen::Vector3d min;
    Eigen::Vector3d max;

    AABB(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

    // Build directly from a YAML node.
    explicit AABB(const YAML::Node& node);

    bool contains(const Eigen::Vector3d& p) const;
    
};

// ------------------------------------------------------------
//  Agent - start/goal pair.
// ------------------------------------------------------------
struct Agent {
    int id;
    Eigen::Vector3d start;

    explicit Agent(const YAML::Node& node);
};

// ------------------------------------------------------------
//  Environment
// ------------------------------------------------------------
class Environment {
public:
    // Environment owns the OcTree (unique_ptr prevents leaks).
    std::unique_ptr<octomap::OcTree> tree;

    Eigen::Vector3d world_min;
    Eigen::Vector3d world_max;

    std::vector<AABB> obstacles;
    std::vector<Agent> agents;      // starts + goals are read from here

    bool labeled;
    std::vector<Eigen::Vector3d> goal_positions; // Used only when labeled=false.

    Environment();

    static Environment loadFromYAML(const std::string& path, double resolution = 0.1);

    bool inBounds(const Eigen::Vector3d& pos) const;
};
