#pragma once

#include "vec2.hpp"

struct Line {
    const Vec2 p1;
    const Vec2 p2;

    Line(const Vec2& p1, const Vec2& p2) : p1(p1), p2(p2) {}
};
