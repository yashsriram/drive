#pragma once

#include "obstacles/line.hpp"
#include "vec2.hpp"

struct Route {
    const std::vector<Vec2> points;

    Route(const std::vector<Vec2>& points) : points(points) {
        if (points.size() < 2) {
            throw std::runtime_error("Attempt to initialize a route with less than 2 points.");
        }
    }

    std::vector<Line> padding_obstacles(float padding) { return {}; }
};
