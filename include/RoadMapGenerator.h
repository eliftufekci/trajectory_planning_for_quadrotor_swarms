#pragma once
#include <map>
#include <tuple>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include "Environment.h"
#include "Graph.h"
#include "RobotModel.h"


class RoadMapGenerator {
public:
    const Environment& env;
    RobotModel robot;
    double grid_step;

    RoadMapGenerator(const Environment& env,
                     const RobotModel& robot,
                     double grid_step = 0.5)
        : env(env), robot(robot), grid_step(grid_step) {}

    Graph createRoadMap() {
        Graph graph;
        std::map<std::tuple<int,int,int>, int> indexMap;

        addFreeVertices(graph, indexMap);
        addEdges(graph, indexMap);
        connectAgents(graph, indexMap);

        return graph;
    }

private:
    // ── Koordinatı tuple index'e çevir ───────────────────────────────
    std::tuple<int,int,int> toKey(double x, double y, double z) const {
        return { static_cast<int>(round(x / grid_step)),
                 static_cast<int>(round(y / grid_step)),
                 static_cast<int>(round(z / grid_step)) };
    }

    std::tuple<int,int,int> toKey(const Eigen::Vector3d& p) const {
        return toKey(p.x(), p.y(), p.z());
    }

    // Tuple index'i gerçek koordinata çevir
    Eigen::Vector3d toPos(const std::tuple<int,int,int>& key) const {
        auto [ix, iy, iz] = key;
        return { ix * grid_step, iy * grid_step, iz * grid_step };
    }

    // ── 1: engel olmayan noktaları vertex olarak ekle ─────────
    void addFreeVertices(Graph& graph,
                         std::map<std::tuple<int,int,int>, int>& indexMap)
    {
        const auto& wmin = env.world_min;
        const auto& wmax = env.world_max;

        for (double x = wmin.x(); x <= wmax.x(); x += grid_step)
        for (double y = wmin.y(); y <= wmax.y(); y += grid_step)
        for (double z = wmin.z(); z <= wmax.z(); z += grid_step) {
            Eigen::Vector3d point(x, y, z);
            if (env.isOccupied(point)) continue;

            auto key = toKey(point);
            if (indexMap.count(key)) continue;     // zaten eklendi

            int id = graph.addVertex(point);
            indexMap[key] = id;
        }
    }

    // ── 2: her vertex için 6 komşuya edge ekle ────────────────
    void addEdges(Graph& graph,
                  std::map<std::tuple<int,int,int>, int>& indexMap)
    {
        // 6 yönlü komşuluk (±x, ±y, ±z)
        const std::vector<Eigen::Vector3d> directions = {
            { grid_step, 0, 0}, {-grid_step, 0, 0},
            {0,  grid_step, 0}, {0, -grid_step, 0},
            {0, 0,  grid_step}, {0, 0, -grid_step},
        };

        for (const auto& [key, id1] : indexMap) {
            Eigen::Vector3d pos = toPos(key);

            for (const auto& dir : directions) {
                Eigen::Vector3d neighbor = pos + dir;
                auto neighborKey = toKey(neighbor);

                // Komşu indexMap'te yoksa engelde veya sınır dışında
                if (!indexMap.count(neighborKey)) continue;

                int id2 = indexMap.at(neighborKey);

                // Çifte kenar eklemeyi önle (id1 < id2)
                if (id1 >= id2) continue;

                if (env.isEdgeFree(pos, neighbor, robot.r_env)) {
                    graph.addEdge(id1, id2);
                }
            }
        }
    }

    // ── 3: Agent start/goal'larını en yakın vertex'e bağla ───────────
    void connectAgents(Graph& graph,
                       std::map<std::tuple<int,int,int>, int>& indexMap)
    {
        for (const auto& agent : env.agents) {
            connectPoint(agent.start, graph, indexMap);
            connectPoint(agent.goal,  graph, indexMap);
        }
    }

    // Nokta indexMap'te yoksa vertex ekle + en yakın vertex'e bağla
    void connectPoint(const Eigen::Vector3d& point,
                      Graph& graph,
                      std::map<std::tuple<int,int,int>, int>& indexMap)
    {
        auto key = toKey(point);
        if (indexMap.count(key)) return;   // zaten grid'de var

        double minDist = std::numeric_limits<double>::max();
        int nearestId = -1;

        for (const auto& [k, vid] : indexMap) {
            double dist = (toPos(k) - point).norm();
            if (dist < minDist) {
                minDist   = dist;
                nearestId = vid;
            }
        }

        int id = graph.addVertex(point);
        indexMap[key] = id;

        if (nearestId != -1) {
            graph.addEdge(id, nearestId);
        }
    }
};