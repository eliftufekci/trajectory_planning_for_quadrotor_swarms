#pragma once
#include <vector>
#include <string>
#include <stdexcept>
#include <octomap/octomap.h>
#include <Eigen/Dense>
#include "Environment.h"
#include "Graph.h"

class RoadMapGenerator{
public:
    Environment env;
    
    RoadMapGenerator(Environment env) : env(env) {}

    Graph createRoadMap() const{

        Graph graph;

        const auto& resolution = env.tree.getResolution();
        
        const auto& min_x = env.world_min.x();
        const auto& min_y = env.world_min.y();
        const auto& min_z = env.world_min.z();

        const auto& max_x = env.world_max.x();
        const auto& max_y = env.world_max.y();
        const auto& max_z = env.world_max.z();

        Eigen::Vector3d point1(min_x, min_y, min_z);
        int id1 = graph.addVertex(point1);
        
        for (double x = min_x + resolution; x <= max_x; x += resolution) {
            for (double y = min_y + resolution; y <= max_y; y += resolution) {
                for (double z = min_z + resolution; z <= max_z; z += resolution) {
                    
                    Eigen::Vector3d point2(x, y, z);
                    if(env.isOccupied(point2)){
                        continue;
                    }
                    
                    id2 = graph.addVertex(point2);
                    if (env.isEdgeFree(point1, point2, env.robot_radius, 0.5)){
                        graph.addEdge(id1, id2);
                    }

                    point1 = point2;
                }
            }
        } 

        return graph;

    }

};