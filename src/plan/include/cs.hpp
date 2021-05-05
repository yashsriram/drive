#pragma once

#include <math.h>

#include "obstacles/circle.hpp"

struct ConfigurationSpace {
    const float agent_radius;
    std::vector<Circle> circles;

    ConfigurationSpace(float agent_radius) : agent_radius(agent_radius) {}

    bool is_colliding(const Vec2& p1, const Vec2& p2) {
        for (const auto& circle : circles) {
            if (circle.is_colliding(p1, p2, agent_radius)) {
                return true;
            }
        }
        return false;
    }
};

