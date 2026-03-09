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
inline Eigen::Vector3d yamlToVec3(const YAML::Node& node) {
    return Eigen::Vector3d(
        node[0].as<double>(),
        node[1].as<double>(),
        node[2].as<double>()
    );
}

// ─────────────────────────────────────────────
//  AABB — Axis-Aligned Bounding Box
// ─────────────────────────────────────────────
struct AABB {
    Eigen::Vector3d min;
    Eigen::Vector3d max;

    AABB(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
        : min(min), max(max) {}

    // YAML node'dan doğrudan oluştur
    explicit AABB(const YAML::Node& node)
        : min(yamlToVec3(node["min"]))
        , max(yamlToVec3(node["max"])) {}

    bool contains(const Eigen::Vector3d& p) const {
        return p.x() >= min.x() && p.x() <= max.x() &&
               p.y() >= min.y() && p.y() <= max.y() &&
               p.z() >= min.z() && p.z() <= max.z();
    }
};

// ─────────────────────────────────────────────
//  Agent — başlangıç / hedef çifti
// ─────────────────────────────────────────────
struct Agent {
    int id;
    Eigen::Vector3d start;
    Eigen::Vector3d goal;

    explicit Agent(const YAML::Node& node)
        : id(node["id"].as<int>())
        , start(yamlToVec3(node["start"]))
        , goal(yamlToVec3(node["goal"])) {}
};

// ─────────────────────────────────────────────
//  Environment
// ─────────────────────────────────────────────
class Environment {
public:
    // OcTree'nin sahipliğini Environment üstleniyor (unique_ptr ile sızıntı yok)
    std::unique_ptr<octomap::OcTree> tree;

    double robot_radius;
    Eigen::Vector3d world_min;
    Eigen::Vector3d world_max;

    std::vector<AABB> obstacles;
    std::vector<Agent> agents;      // starts + goals buradan okunur

    // ── Varsayılan constructor (loadFromYAML kullanımı için) ─────────────
    Environment() : tree(nullptr), robot_radius(0.1) {}

    // ── YAML'dan yükle — static factory ──────────────────────────────────
    static Environment loadFromYAML(const std::string& path, double resolution = 0.1) {
        YAML::Node config = YAML::LoadFile(path);
        Environment env;

        // Dünya sınırları
        env.world_min = yamlToVec3(config["world_bounds"]["min"]);
        env.world_max = yamlToVec3(config["world_bounds"]["max"]);

        // OcTree oluştur
        env.tree = std::make_unique<octomap::OcTree>(resolution);

        // Engelleri oku ve OcTree'ye voxel olarak ekle
        for (const auto& node : config["obstacles"]) {
            AABB aabb(node);
            env.obstacles.push_back(aabb);

            // AABB içindeki tüm voxel'leri dolu işaretle
            for (double x = aabb.min.x(); x <= aabb.max.x(); x += resolution) {
                for (double y = aabb.min.y(); y <= aabb.max.y(); y += resolution) {
                    for (double z = aabb.min.z(); z <= aabb.max.z(); z += resolution) {
                        //                          ↑ burada x değil z!
                        env.tree->updateNode(octomap::point3d(
                            static_cast<float>(x),
                            static_cast<float>(y),
                            static_cast<float>(z)), true);
                    }
                }
            }
        }
        env.tree->updateInnerOccupancy();

        // Agent'ları oku
        if (config["agents"]) {
            for (const auto& node : config["agents"]) {
                env.agents.emplace_back(node);
            }
        }

        return env;   // RVO/move semantics — kopyalama yok
    }

    // ── Nokta boş mu? ──────────────────────────────────────────────────
    bool isOccupied(const Eigen::Vector3d& pos) const {
        auto* node = tree->search(pos.x(), pos.y(), pos.z());
        return node && tree->isNodeOccupied(node);
    }

    // ── Dünya sınırları içinde mi? ─────────────────────────────────────
    bool inBounds(const Eigen::Vector3d& pos) const {
        return pos.x() >= world_min.x() && pos.x() <= world_max.x() &&
               pos.y() >= world_min.y() && pos.y() <= world_max.y() &&
               pos.z() >= world_min.z() && pos.z() <= world_max.z();
    }

    // ── İki nokta arasındaki kenar engel içinden geçiyor mu? ───────────
    // robot_radius: Minkowski genişlemesi için robot yarıçapı
    // step_ratio  : adım büyüklüğü = resolution * step_ratio (0 < ratio ≤ 1)
    bool isEdgeFree(const Eigen::Vector3d& a,
                    const Eigen::Vector3d& b,
                    double r_robot,
                    double step_ratio = 0.5) const
    {
        const double resolution = tree->getResolution();
        const double step = resolution * step_ratio;
        const Eigen::Vector3d dir = b - a;
        const double length = dir.norm();

        if (length < 1e-9) return !isOccupied(a);  // aynı nokta

        const Eigen::Vector3d unit = dir / length;

        // Ray üzerinde adım adım örnekle
        for (double t = 0.0; t <= length; t += step) {
            Eigen::Vector3d p = a + t * unit;

            // Robot hacmini temsil etmek için küresel örnekleme
            // (tam Minkowski için FCL kullanılabilir — bkz. makale Sec. IV)
            for (double dx : {-r_robot, 0.0, r_robot}) {
                for (double dy : {-r_robot, 0.0, r_robot}) {
                    for (double dz : {-r_robot, 0.0, r_robot}) {
                        Eigen::Vector3d sample = p + Eigen::Vector3d(dx, dy, dz);
                        if (!inBounds(sample) || isOccupied(sample))
                            return false;
                    }
                }
            }
        }
        return true;
    }
};