#pragma once
#include <vector>
#include <string>
#include <stdexcept>
#include <octomap/octomap.h>
#include <Eigen/Dense>
#include "Environment.h"
#include "Graph.h"
#include "RobotModel.h"
#include <map>
#include <tuple>


class RoadMapGenerator{
public:
    const Environment& env;
    RobotModel robotModel;
    
    RoadMapGenerator(const Environment& env, RobotModel robotModel) : env(env), robotModel(robotModel) {}
    
    Graph createRoadMap(){
        
        Graph graph;
        std::map<std::tuple<int,int,int>, int> indexMap;
        
        const auto& min_x = env.world_min.x();
        const auto& min_y = env.world_min.y();
        const auto& min_z = env.world_min.z();

        const auto& max_x = env.world_max.x();
        const auto& max_y = env.world_max.y();
        const auto& max_z = env.world_max.z();
        
        double grid_step = 0.5;
        
        for (double x = min_x; x <= max_x; x += grid_step) {
            for (double y = min_y; y <= max_y; y += grid_step) {
                for (double z = min_z; z <= max_z; z += grid_step) {
                    
                    Eigen::Vector3d point(x, y, z);
                    if(env.isOccupied(point)){
                        continue;
                    }
                    
                    if(indexMap.count(point) != 0){
                        continue;
                    }   

                    int id = graph.addVertex(point);
                    indexMap[make_tuple( round(point.x()/grid_step),
                                         round(point.y()/grid_step),
                                         round(point.z()/grid_step))] = id;
                }
            }
        }

        for (double x = min_x; x <= max_x; x += grid_step) {
            for (double y = min_y; y <= max_y; y += grid_step) {
                for (double z = min_z; z <= max_z; z += grid_step) {
                
                    int id1 = indexMap.at(make_tuple(   round(point.x()/grid_step),
                                                        round(point.y()/grid_step),
                                                        round(point.z()/grid_step)));

                    std::vector<Eigen::Vector3d> neighbors;
                    neighbors.emplace_back(x+grid_step, y, z);
                    neighbors.emplace_back(x-grid_step, y, z);
                    neighbors.emplace_back(x, y+grid_step, z);
                    neighbors.emplace_back(x, y-grid_step, z);
                    neighbors.emplace_back(x, y, z+grid_step);
                    neighbors.emplace_back(x, y, z-grid_step);

                    for(const auto& neighbor : neighbors){
                        
                        int id2 = indexMap.at(make_tuple(   neighbor(point.x()/grid_step),
                                                            neighbor(point.y()/grid_step),
                                                            neighbor(point.z()/grid_step)));

                        if(env.isEdgeFree(Eigen::Vector3d(x,y,z), neighbor, robotModel.r_env, grid_step)){
                            graph.addEdge(id1, id2);
                        }                              
                    }
                }
            }
        }     

        for(const auto& agent : env.agents){
            auto pos = make_tuple(  round(agent.start.x()/grid_step),
                                    round(agent.start.y()/grid_step),
                                    round(agent.start.z()/grid_step))

            if(indexMap.count(pos) == 0){
                int id = graph.addVertex(agent.start);
                indexMap.at(pos) = id; 

                auto min = INT_MAX;
                int min_id;

                for(const auto& key : index_map){

                    auto dist = (Eigen::Vector3d(key.first) - Eigen::Vector3d(pos)).norm();
                    if(min > dist){
                        dist = min;
                        min_id = key.second;
                    }
                }

                graph.addEdge(id, indexMap.at(min_id));
            }

            
            pos = make_tuple(   round(agent.goal.x()/grid_step),
                                round(agent.goal.y()/grid_step),
                                round(agent.goal.z()/grid_step))

            if(indexMap.count(pos) == 0){
                int id = graph.addVertex(agent.goal);
                indexMap.at(pos) = id; 

                auto min = INT_MAX;
                int min_id;

                for(const auto& key : index_map){

                    auto dist = (Eigen::Vector3d(key.first) - Eigen::Vector3d(pos)).norm();
                    if(min > dist){
                        dist = min;
                        min_id = key.second;
                    }
                }

                graph.addEdge(id, indexMap.at(min_id));
            }
            
        } 

        return graph;

    }
};