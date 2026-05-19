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
    // Convert a coordinate to a tuple index.
    std::tuple<int,int,int> toKey(double x, double y, double z) const ;

    std::tuple<int,int,int> toKey(const Eigen::Vector3d& p) const;

    // Convert a tuple index to a real coordinate.
    Eigen::Vector3d toPos(const std::tuple<int,int,int>& key) const;

    // 1: Add non-obstacle points as vertices.
    void addFreeVertices(Graph& graph, std::map<std::tuple<int,int,int>, int>& indexMap);

    // 2: Add edges to 6 neighbors for each vertex.
    void addEdges(Graph& graph, std::map<std::tuple<int,int,int>, int>& indexMap);

    // 3: Connect agent starts/goals to the nearest vertex.
    void connectAgents(Graph& graph, std::map<std::tuple<int,int,int>, int>& indexMap);

    // If the point is not in indexMap, add a vertex and connect it to the nearest 6 vertices.
    int connectPoint(const Eigen::Vector3d& point,
                    Graph& graph,
                    std::map<std::tuple<int,int,int>, int>& indexMap);
};
