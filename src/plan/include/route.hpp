#pragma once

#include "vec2.hpp"

struct Route {
    const std::vector<Vec2> points;

    Route(const std::vector<Vec2>& points) : points(points) {
        if (points.size() < 2) {
            throw std::runtime_error("Attempt to initialize a route with less than 2 points.");
        }
        for (int i = 1; i < points.size(); ++i) {
            if (points[i - 1] == points[i]) {
                throw std::runtime_error("Two consecutive points on route are equal.");
            }
        }
    }
};
