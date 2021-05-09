#pragma once

#include "sense/obstacles/line.hpp"
#include "vec2.hpp"

struct Rectangle {
    const Vec2 center;
    const float orientation;
    const float width;
    const float height;
    const Line l1;
    const Line l2;
    const Line l3;
    const Line l4;

    Rectangle(const Vec2& center, const float orientation, const float width, const float height)
        : center(center),
          orientation(orientation),
          width(width),
          height(height),
          l1(Line(center + Vec2(-width / 2, height / 2).rotate(orientation), center + Vec2(width / 2, height / 2).rotate(orientation))),
          l2(Line(center + Vec2(width / 2, height / 2).rotate(orientation), center + Vec2(width / 2, -height / 2).rotate(orientation))),
          l3(Line(center + Vec2(width / 2, -height / 2).rotate(orientation), center + Vec2(-width / 2, -height / 2).rotate(orientation))),
          l4(Line(center + Vec2(-width / 2, -height / 2).rotate(orientation), center + Vec2(-width / 2, height / 2).rotate(orientation))) {
        if (width <= 0 || height <= 0) {
            throw std::runtime_error("Bad obstacle width or height.");
        }
    }

    // Does not return true if both p1 and p2 are inside rectangle
    bool does_intersect(const Vec2& p1, const Vec2& p2) const {
        if (l1.does_intersect(p1, p2) || l2.does_intersect(p1, p2) || l3.does_intersect(p1, p2) || l4.does_intersect(p1, p2)) {
            return true;
        }
        return false;
    }
};
