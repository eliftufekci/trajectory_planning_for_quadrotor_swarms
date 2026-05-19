#pragma once
#include <vector>
#include <string>
#include <stdexcept>
#include <octomap/octomap.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>


// ─────────────────────────────────────────────
//  Yardımcı: YAML dizisini Vector3d'ye çevir
// ─────────────────────────────────────────────
inline Eigen::Vector3d yamlToVec3(const YAML::Node& node);

// ─────────────────────────────────────────────
//  AABB — Axis-Aligned Bounding Box
// ─────────────────────────────────────────────
struct AABB {
    Eigen::Vector3d min;
    Eigen::Vector3d max;

    AABB(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

    // YAML node'dan doğrudan oluştur
    explicit AABB(const YAML::Node& node);

    bool contains(const Eigen::Vector3d& p) const;
    
};

// ─────────────────────────────────────────────
//  Agent — başlangıç / hedef çifti
// ─────────────────────────────────────────────
struct Agent {
    int id;
    Eigen::Vector3d start;

    explicit Agent(const YAML::Node& node);
};

// ─────────────────────────────────────────────
//  Environment
// ─────────────────────────────────────────────
class Environment {
public:
    // OcTree'nin sahipliğini Environment üstleniyor (unique_ptr ile sızıntı yok)
    std::unique_ptr<octomap::OcTree> tree;

    Eigen::Vector3d world_min;
    Eigen::Vector3d world_max;

    std::vector<AABB> obstacles;
    std::vector<Agent> agents;      // starts + goals buradan okunur

    bool labeled;
    std::vector<Eigen::Vector3d> goal_positions; // Sadece labeled=false ise kullanılır

    Environment();

    static Environment loadFromYAML(const std::string& path, double resolution = 0.1);

    bool inBounds(const Eigen::Vector3d& pos) const;
};