#pragma once

#include <eigen3/Eigen/Dense>

#include "vec2.hpp"

struct VisibilityVertex {
    const int id;
    const Vec2 position;
    std::vector<int> neighbourIds;

    VisibilityVertex(const int id, const float x, const float y) : id(id), position(x, y) { reset_search_state(); }

    void add_neighbour(const int id) { neighbourIds.push_back(id); }

    bool is_explored = false;
    float distance_from_start = 0;
    float distance_to_finish = 0;
    std::vector<int> path_from_start;
    Eigen::Vector3f color;

    void reset_search_state() {
        is_explored = false;
        distance_from_start = 0;
        distance_to_finish = 0;
        path_from_start.clear();
        color = Eigen::Vector3f(1.0, 1.0, 1.0);
    }
};

