#pragma once

#include "vec3.hpp"

struct Obstacle {
    const Vec3 center;
    const float radius;

    Obstacle(const Vec3& center, float radius) : center(center), radius(radius) {
        if (radius < 0) {
            throw std::runtime_error("Bad obstacle radius");
        }
        if (center.z != 0) {
            throw std::runtime_error("Flying obstacle");
        }
    }
};
