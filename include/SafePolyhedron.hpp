#pragma once
#include <vector>

struct HyperPlane;

struct SafePolyhedron{
    std::vector<std::vector<std::vector<HyperPlane>>> planes;

    SafePolyhedron(std::vector<std::vector<std::vector<HyperPlane>>> planes);
};