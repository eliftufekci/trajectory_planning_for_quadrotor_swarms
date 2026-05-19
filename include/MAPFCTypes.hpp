#pragma once
#include <iostream>
#include <vector>
#include <unordered_set>
#include <tuple>
#include "Graph.hpp"

struct State {
    int time;
    int vertex_id;

    State(int time, int vertex_id);

    bool operator==(const State& s) const;
    bool equalExceptTime(const State& s) const;

    friend std::ostream& operator<<(std::ostream& os, const State& s);
};

namespace std {
    template <>
    struct hash<State> {
        size_t operator()(const State& s) const;
    };
}

struct Action { 
    int edge_id;
    Action(int edge_id);
    // wait: -1
};

struct Conflict {
    enum Type {
        vertex,
        edge,
        conVV,
        conEE,
        conEV
    };

    size_t agent1;
    size_t agent2;
    int time;
    Type type;

    int vertex_id; // for conVV
    int edge_id1;  // for conEE and conEV
    int edge_id2;

    friend std::ostream& operator<<(std::ostream& os, const Conflict& c);
};

struct VertexConstraint {
    int time;
    int vertex_id;

    VertexConstraint(int time, int vertex_id);

    bool operator==(const VertexConstraint& vc) const;
    bool operator<(const VertexConstraint& other) const;
};

namespace std {
    template <>
    struct hash<VertexConstraint> {
        size_t operator()(const VertexConstraint& vc) const;
    };
}

struct EdgeConstraint {
    int time;
    int edge_id;

    EdgeConstraint(int time, int edge_id);

    bool operator==(const EdgeConstraint& ec) const;
    bool operator<(const EdgeConstraint& other) const;
};

namespace std {
    template <>
    struct hash<EdgeConstraint> {
        size_t operator()(const EdgeConstraint& ec) const;
    };
}

struct Constraints {
    std::unordered_set<VertexConstraint> vertexConstraints;
    std::unordered_set<EdgeConstraint> edgeConstraints;

    void add(const Constraints& other);
    bool overlap(const Constraints& other) const;
};
