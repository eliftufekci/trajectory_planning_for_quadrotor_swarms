#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>

// Axis-Aligned Bounding Box obstacle
// min = alt-sol-arka köşe, max = üst-sağ-ön köşe
struct AABB{
    Eigen::Vector3d min;
    Eigen::Vector3d max;

    AABB(min, max): min(min), max(max) {}

    bool contains(const Eigen::Vector& p) const{
        return p.x() >= min.x() && p.x() <= max.x() &&
               p.y() >= min.y() && p.y() <= max.y() &&
               p.z() >= min.z() && p.z() <= max.z();
    }
};

class Environment{
public:
    Eigen::Vector3d workspace_min;
    Eigen::Vector3d workspace_max;
    std::vector<AABB> obstacles; 

    Environment(workspace_min, workspace_max, obstacles) : workspace_min(workspace_min), workspace_max(workspace_max), obstacles(obstacles) {}


    void addObstacle(Eigen::Vector3d min, Eigen::Vector3d max) {
        obstacles.emplace_back(min, max);
        // nesneyi vektörün içinde oluşturur 
    }

    bool isCollision(const Eigen::Vector3d p) const{
        if(p.x() < min.x() || p.x() > max.x() ||
           p.y() < min.y() || p.y() > max.y() ||
           p.z() < min.z() || p.z() > max.z()){
            
           return true
        }

        for(const obs : obstacles){
            if(obs.contains(p)) return true;
        } 

        return false;
    }

    boolean isEdge Collision

};