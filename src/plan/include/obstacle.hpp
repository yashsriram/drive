#pragma once

#include "vec2.hpp"

struct Obstacle {
    const Vec2 center;
    const float radius;

    Obstacle(const Vec2& center, float radius) : center(center), radius(radius) {
        if (radius < 0) {
            throw std::runtime_error("Bad obstacle radius");
        }
    }
};
