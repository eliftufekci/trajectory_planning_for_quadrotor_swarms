#pragma once
#include <map>
#include <tuple>
#include <vector>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>
#include "Environment.hpp"
#include "FCLCollisionChecker.hpp"
#include "Graph.hpp"
#include "RobotModel.hpp"
#include "RoadMapGenerator.hpp"

class GridRoadMapGenerator : public RoadMapGenerator{
public:
    double grid_step;

    GridRoadMapGenerator(const Environment& env,
                     const RobotModel& robot,
                     const FCLCollisionChecker& collisionChecker,
                     double grid_step = 0.5);

    Graph createRoadMap();

private:
    // ── Koordinatı tuple index'e çevir ───────────────────────────────
    std::tuple<int,int,int> toKey(double x, double y, double z) const ;

    std::tuple<int,int,int> toKey(const Eigen::Vector3d& p) const;

    // Tuple index'i gerçek koordinata çevir
    Eigen::Vector3d toPos(const std::tuple<int,int,int>& key) const;

    // ── 1: engel olmayan noktaları vertex olarak ekle ─────────
    void addFreeVertices(Graph& graph, std::map<std::tuple<int,int,int>, int>& indexMap);

    // ── 2: her vertex için 6 komşuya edge ekle ────────────────
    void addEdges(Graph& graph, std::map<std::tuple<int,int,int>, int>& indexMap);

    // ── 3: Agent start/goal'larını en yakın vertex'e bağla ───────────
    void connectAgents(Graph& graph, std::map<std::tuple<int,int,int>, int>& indexMap);

    // Nokta indexMap'te yoksa vertex ekle + en yakın 6 vertex'e bağla
    int connectPoint(const Eigen::Vector3d& point,
                    Graph& graph,
                    std::map<std::tuple<int,int,int>, int>& indexMap);
};