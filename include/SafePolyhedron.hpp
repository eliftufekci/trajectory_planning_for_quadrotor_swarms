#pragma once
#include <vector>

struct SafePolyhedron{
    std::vector<HyperPlane> planes;

    SafePolyhedron(std::vector<HyperPlane> planes) : planes(planes){}
    
};