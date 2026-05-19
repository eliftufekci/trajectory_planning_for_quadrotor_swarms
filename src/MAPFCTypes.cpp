#include "MAPFCTypes.hpp"

// State implementation.
State::State(int time, int vertex_id) : time(time), vertex_id(vertex_id) {}

bool State::operator==(const State& s) const {
    return time == s.time && vertex_id == s.vertex_id;
}

bool State::equalExceptTime(const State& s) const {
    return vertex_id == s.vertex_id; 
}

std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << ": (" << s.vertex_id << ")";
}

// Rule: explicitly use the 'std::' namespace in .cpp files.
namespace std {
    size_t hash<State>::operator()(const State& s) const {
        std::hash<int> hasher;
        size_t seed = hasher(s.time);
        seed ^= hasher(s.vertex_id) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
}

// Action implementation.
Action::Action(int edge_id) : edge_id(edge_id) {}

// Conflict implementation.
// Rule: the 'friend' keyword is not used in .cpp files; write it as a regular function.
std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    switch (c.type) {
        case Conflict::vertex:
            return os << c.time << ": Vertex(" << c.vertex_id << ")";
        case Conflict::edge:
            return os << c.time << ": Edge(" << c.edge_id1 << ")";
        case Conflict::conVV:
            return os << c.time << ": ConVV(" << c.vertex_id << ")";
        case Conflict::conEE:
            return os << c.time << ": ConEE(" << c.edge_id1 << ", " << c.edge_id2 << ")";
        case Conflict::conEV:
            return os << c.time << ": ConEV(" << c.edge_id1 << "," << c.vertex_id << ")";
    }
    return os;
}

// VertexConstraint implementation.
VertexConstraint::VertexConstraint(int time, int vertex_id) : time(time), vertex_id(vertex_id) {}

bool VertexConstraint::operator==(const VertexConstraint& vc) const {
    return time == vc.time && vertex_id == vc.vertex_id;
}

bool VertexConstraint::operator<(const VertexConstraint& other) const {
    return std::tie(time, vertex_id) < std::tie(other.time, other.vertex_id);
}

namespace std {
    size_t hash<VertexConstraint>::operator()(const VertexConstraint& vc) const {
        std::hash<int> hasher;
        size_t seed = hasher(vc.time);
        seed ^= hasher(vc.vertex_id) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
}

// EdgeConstraint implementation.
EdgeConstraint::EdgeConstraint(int time, int edge_id) : time(time), edge_id(edge_id) {}

bool EdgeConstraint::operator==(const EdgeConstraint& ec) const {
    return time == ec.time && edge_id == ec.edge_id;
}

bool EdgeConstraint::operator<(const EdgeConstraint& other) const {
    return std::tie(time, edge_id) < std::tie(other.time, other.edge_id);
}

namespace std {
    size_t hash<EdgeConstraint>::operator()(const EdgeConstraint& ec) const {
        std::hash<int> hasher;
        size_t seed = hasher(ec.time);
        seed ^= hasher(ec.edge_id) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
}

// Constraints implementation.
void Constraints::add(const Constraints& other) {
    vertexConstraints.insert(other.vertexConstraints.begin(), other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(), other.edgeConstraints.end());
}

bool Constraints::overlap(const Constraints& other) const {
    for (const auto& vc : vertexConstraints) {
        if (other.vertexConstraints.count(vc) > 0) {
            return true;
        }
    }
    for (const auto& ec : edgeConstraints) {
        if (other.edgeConstraints.count(ec) > 0) {
            return true;
        }
    }
    return false;
}
