#include "SafePolyhedron.hpp"
#include "HyperPlane.hpp"

SafePolyhedron::SafePolyhedron(std::vector<std::vector<std::vector<HyperPlane>>> planes) : planes(planes) {}