#pragma once
#include <vector>

struct SafePolyhedron{
    std::vector<std::vector<std::vector<HyperPlane>>> planes;

    SafePolyhedron(std::vector<std::vector<std::vector<HyperPlane>>> planes) : planes(planes){}
    
};