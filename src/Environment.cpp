#include <vector>
#include <string>
#include <stdexcept>
#include <octomap/octomap.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "Environment.hpp"

inline Eigen::Vector3d yamlToVec3(const YAML::Node& node) {
    return Eigen::Vector3d(
        node[0].as<double>(),
        node[1].as<double>(),
        node[2].as<double>()
    );
}

AABB::AABB(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
    : min(min), max(max) {}

// Build directly from a YAML node.
AABB::AABB(const YAML::Node& node)
    : min(yamlToVec3(node["min"]))
    , max(yamlToVec3(node["max"])) {}

bool AABB::contains(const Eigen::Vector3d& p) const {
    return p.x() >= min.x() && p.x() <= max.x() &&
            p.y() >= min.y() && p.y() <= max.y() &&
            p.z() >= min.z() && p.z() <= max.z();
}

Agent::Agent(const YAML::Node& node)
    : id(node["id"].as<int>())
    , start(yamlToVec3(node["start"])) {}

// Default constructor (for loadFromYAML usage).
Environment::Environment() : tree(nullptr) {}

// Load from YAML - static factory.
Environment Environment::loadFromYAML(const std::string& path, double resolution) {
    YAML::Node config = YAML::LoadFile(path);
    Environment env;

    // World bounds.
    env.world_min = yamlToVec3(config["world_bounds"]["min"]);
    env.world_max = yamlToVec3(config["world_bounds"]["max"]);

    // Create the OcTree.
    env.tree = std::make_unique<octomap::OcTree>(resolution);

    // Read obstacles and add them to the OcTree as voxels.
    for (const auto& node : config["obstacles"]) {
        AABB aabb(node);
        env.obstacles.push_back(aabb);

        // Mark all voxels inside the AABB as occupied.
        for (double x = aabb.min.x(); x <= aabb.max.x(); x += resolution) {
            for (double y = aabb.min.y(); y <= aabb.max.y(); y += resolution) {
                for (double z = aabb.min.z(); z <= aabb.max.z(); z += resolution) {
                    env.tree->updateNode(octomap::point3d(
                        static_cast<float>(x),
                        static_cast<float>(y),
                        static_cast<float>(z)), true);
                }
            }
        }
    }
    env.tree->updateInnerOccupancy();

    env.labeled = config["labeled"] ? config["labeled"].as<bool>() : true;

    if (config["goals"])
        for (const auto& g : config["goals"])
            env.goal_positions.push_back(yamlToVec3(g));

    // Goals are no longer stored while reading agents.
    if (config["agents"])
        for (const auto& node : config["agents"])
            env.agents.emplace_back(node);

    return env;   
}


// Is the position inside the world bounds?
bool Environment::inBounds(const Eigen::Vector3d& pos) const {
    return pos.x() >= world_min.x() && pos.x() <= world_max.x() &&
            pos.y() >= world_min.y() && pos.y() <= world_max.y() &&
            pos.z() >= world_min.z() && pos.z() <= world_max.z();
}
