#include <iostream>
#include <fstream>
#include "Environment.hpp"
#include "FCLCollisionChecker.hpp"
#include "Graph.hpp"
#include "HyperPlaneSeparator.hpp"
#include "RobotModel.hpp"
#include "GridRoadMapGenerator.hpp"
#include "SPARSRoadMapGenerator.hpp"
#include "FCLConflictAnnotation.hpp"
#include "SafePolyhedron.hpp"
#include "SweptConflictAnnotation.hpp"
#include "MAPFCSolver.hpp"

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

void saveSafePolyhedronCSV(const SafePolyhedron& poly, const std::string& path) {
    std::ofstream f(path);
    if (!f) {
        std::cerr << "HATA: Güvenli Polyhedron dosyası açılamadı: " << path << std::endl;
        return;
    }
    f << "robot_id,timestep,nx,ny,nz,d,ellipsoid_offset\n";
    for (size_t i = 0; i < poly.planes.size(); ++i) {
        for (size_t k = 0; k < poly.planes[i].size(); ++k) {
            for (const auto& plane : poly.planes[i][k]) {
                f << i << "," << k << ","
                  << plane.normal_vector.x() << ","
                  << plane.normal_vector.y() << ","
                  << plane.normal_vector.z() << ","
                  << plane.d << ","
                  << plane.ellipsoid_offset << "\n";
            }
        }
    }
    std::cout << "Kaydedildi: " << path << "\n";
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

    // GridRoadMapGenerator roadMapGenerator(env, fclCollisionChecker, step_size);
    SPARSRoadMapGenerator roadMapGenerator(env, fclCollisionChecker);

    Graph environment_graph;
    try {
        environment_graph = roadMapGenerator.createRoadMap();
    } catch (const std::runtime_error& e) {
        std::cerr << "\n!!! Roadmap olusturma basarisiz !!!\n" << e.what() << std::endl;
        return 1;
    }

    environment_graph.saveToCSV("vertices.csv", "edges.csv");

    std::cout << "Dünya min : " << env.world_min.transpose() << "\n";
    std::cout << "Dünya max : " << env.world_max.transpose() << "\n";
    std::cout << "Engel     : " << env.obstacles.size() << "\n";
    std::cout << "Agent     : " << env.agents.size()    << "\n\n";


    // CSV çıktıları
    saveEnvironmentCSV(env, "environment_data.csv");
    saveOctreeCSV(env, "octree_voxels.csv");

    FCLConflictAnnotation fcl_conflict_annotation(environment_graph, robot);
    fcl_conflict_annotation.annotate();

    SweptConflictAnnotation swept_conflict_annotation(environment_graph, robot);
    swept_conflict_annotation.annotate();

    std::cout << "fcl conVV boyutu: " << fcl_conflict_annotation.conVV.size() << "\n";
    std::cout << "fcl conEV boyutu: " << fcl_conflict_annotation.conEV.size() << "\n";
    std::cout << "fcl conEE boyutu: " << fcl_conflict_annotation.conEE.size() << "\n";

    std::cout << "swept conVV boyutu: " << swept_conflict_annotation.conVV.size() << "\n";
    std::cout << "swept conEV boyutu: " << swept_conflict_annotation.conEV.size() << "\n";
    std::cout << "swept conEE boyutu: " << swept_conflict_annotation.conEE.size() << "\n";

    std::cout << "\n=== Multi-Agent Path Finding (ECBS) ===\n";
    // Çözücüyü swept_conflict_annotation ile başlatıyoruz (fcl de kullanabilirsiniz)
    MPAFCSolver solver(environment_graph, swept_conflict_annotation);
    std::cout << "mapfc solver worked succesfully\n";
    DiscreteSchedule schedule = solver.solve();

    HyperPlaneSeparator hyperPlaneSeparator(environment_graph, robot, schedule, env);
    SafePolyhedron safePolyhedron = hyperPlaneSeparator.compute();
    saveSafePolyhedronCSV(safePolyhedron, "hyperplanes.csv");


    std::cout << "Makespan (K): " << schedule.K << "\n";
    for (size_t i = 0; i < schedule.waypoint.size(); ++i) {
        std::cout << "Robot " << i << " path length: " << schedule.waypoint[i].size() << "\n";
    }

    return 0;
}