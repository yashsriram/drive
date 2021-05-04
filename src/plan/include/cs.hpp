#pragma once

#include <math.h>

#include "obstacle.hpp"

struct ConfigurationSpace {
    const float agent_radius;
    std::vector<Obstacle> obstacles;

    ConfigurationSpace(float agent_radius) : agent_radius(agent_radius) {}

    bool is_colliding(const Vec2& point, const Obstacle& obstacle) { return (point - obstacle.center).norm() <= obstacle.radius + agent_radius; }

    bool is_colliding(const Vec2& p1, const Vec2& p2) {
        for (const auto& obstacle : obstacles) {
            if (is_colliding(p1, obstacle) || is_colliding(p2, obstacle)) {
                return true;
            }
            Vec2 pb_pa = p2 - p1;
            Vec2 pa_pc = p1 - obstacle.center;
            float r = obstacle.radius + agent_radius;
            float a = pb_pa.dot(pb_pa);
            float c = pa_pc.dot(pa_pc) - r * r;
            float b = 2 * pb_pa.dot(pa_pc);
            float discriminant = b * b - 4 * a * c;
            if (discriminant >= 0) {
                float t1 = (float)((-b + sqrt(discriminant)) / (2 * a));
                float t2 = (float)((-b - sqrt(discriminant)) / (2 * a));
                // Intersection with line segment only possible iff at least one of the solutions lies in [0, 1]
                if ((0 <= t1 && t1 <= 1) || (0 <= t2 && t2 <= 1)) {
                    return true;
                }
            }
        }
        return false;
    }
};

