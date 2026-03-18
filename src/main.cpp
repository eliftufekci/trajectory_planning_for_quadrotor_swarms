#include <iostream>
#include <fstream>
#include "Environment.h"
#include "FCLCollisionChecker.h"
#include "Graph.h"
#include "RobotModel.h"
#include "GridRoadMapGenerator.h"
#include "SPARSRoadMapGenerator.h"
#include "FCLConflictAnnotation.h"
#include "SweptConflictAnnotation.h"

void saveEnvironmentCSV(const Environment& env, const std::string& path) {
    std::ofstream f(path);
    f << "type,x,y,z,extra\n";
    f << "world_min," << env.world_min.x() << "," << env.world_min.y() << "," << env.world_min.z() << ",0\n";
    f << "world_max," << env.world_max.x() << "," << env.world_max.y() << "," << env.world_max.z() << ",0\n";
    for (size_t i = 0; i < env.obstacles.size(); ++i) {
        const auto& obs = env.obstacles[i];
        f << "obs_min," << obs.min.x() << "," << obs.min.y() << "," << obs.min.z() << "," << i << "\n";
        f << "obs_max," << obs.max.x() << "," << obs.max.y() << "," << obs.max.z() << "," << i << "\n";
    }
    for (const auto& ag : env.agents) {
        f << "start," << ag.start.x() << "," << ag.start.y() << "," << ag.start.z() << "," << ag.id << "\n";
        f << "goal,"  << ag.goal.x()  << "," << ag.goal.y()  << "," << ag.goal.z()  << "," << ag.id << "\n";
    }
    std::cout << "Kaydedildi: " << path << "\n";
}

void saveOctreeCSV(const Environment& env, const std::string& path) {
    std::ofstream f(path);
    f << "x,y,z\n";
    int count = 0;
    for (auto it = env.tree->begin_leafs(); it != env.tree->end_leafs(); ++it) {
        if (env.tree->isNodeOccupied(*it)) {
            f << it.getX() << "," << it.getY() << "," << it.getZ() << "\n";
            ++count;
        }
    }
    std::cout << "Kaydedildi: " << path << " (" << count << " voxel)\n";
}

int main(int argc, char* argv[]) {
    // YAML path: argümandan al, yoksa varsayılan
    std::string yaml_path = (argc > 1) ? argv[1] : "../config/environment.yaml";

    std::cout << "=== Environment Test ===\n";
    std::cout << "YAML: " << yaml_path << "\n\n";

    Environment env = Environment::loadFromYAML(yaml_path);
    RobotModel robot;
    double step_size = 0.5;
    FCLCollisionChecker fclCollisionChecker(env, robot);
    GridRoadMapGenerator gridRoadMapGenerator(env, fclCollisionChecker, step_size);
    Graph environment_graph = gridRoadMapGenerator.createRoadMap();
    // SPARSRoadMapGenerator sparsRoadMapGenerator(env, fclCollisionChecker);
    // Graph environment_graph = sparsRoadMapGenerator.createRoadMap();

    environment_graph.saveToCSV("vertices.csv", "edges.csv");

    std::cout << "Dünya min : " << env.world_min.transpose() << "\n";
    std::cout << "Dünya max : " << env.world_max.transpose() << "\n";
    std::cout << "Engel     : " << env.obstacles.size() << "\n";
    std::cout << "Agent     : " << env.agents.size()    << "\n\n";


    // CSV çıktıları
    saveEnvironmentCSV(env, "environment_data.csv");
    saveOctreeCSV(env, "octree_voxels.csv");

    auto fcl_conflict_annotation = FCLConflictAnnotation(environment_graph, robot).annotate();
    auto swept_conflict_annotation = SweptConflictAnnotation(environment_graph, robot).annotate();

    std::cout << "fcl conVV boyutu: " << fcl_conflict_annotation.conVV.size() << "\n";
    std::cout << "fcl conEV boyutu: " << fcl_conflict_annotation.conEV.size() << "\n";
    std::cout << "fcl conEE boyutu: " << fcl_conflict_annotation.conEE.size() << "\n";

    std::cout << "swept conVV boyutu: " << swept_conflict_annotation.conVV.size() << "\n";
    std::cout << "swept conEV boyutu: " << swept_conflict_annotation.conEV.size() << "\n";
    std::cout << "swept conEE boyutu: " << swept_conflict_annotation.conEE.size() << "\n";

    return 0;
}