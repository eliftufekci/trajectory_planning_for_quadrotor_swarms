#include "HyperPlane.hpp"

HyperPlane::HyperPlane(Eigen::Vector3d normal_vector, double d, double ellipsoid_offset, int type, int from_id)
    : normal_vector(normal_vector), d(d), ellipsoid_offset(ellipsoid_offset)
    , separated_from_type(type), separated_from_id(from_id) {}