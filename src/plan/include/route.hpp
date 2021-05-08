#pragma once

#include "vec2.hpp"

struct Route {
    const std::vector<Vec2> points;
    int current_point_idx;

    Route(const std::vector<Vec2>& points) : points(points), current_point_idx(0) {
        if (points.size() < 2) {
            throw std::runtime_error("Attempt to initialize a route with less than 2 points.");
        }
        for (int i = 1; i < points.size(); ++i) {
            if (points[i - 1] == points[i]) {
                throw std::runtime_error("Two consecutive points on route are equal.");
            }
        }
    }

    const Vec2& start() const { return points[0]; }

    const Vec2& current_goal() const {
        if (current_point_idx < points.size() - 1) {
            return points[current_point_idx + 1];
        } else {
            return points[current_point_idx];
        }
    }

    void increment_goal() {
        if (current_point_idx < points.size() - 1) {
            current_point_idx++;
        }
    }

    bool is_done() { return current_point_idx == points.size() - 1; }
};
