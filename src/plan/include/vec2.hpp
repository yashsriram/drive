#pragma once

#include <cmath>
#include <ostream>

struct Vec2 {
    float x, y;

    Vec2(const float x, const float y) : x(x), y(y) {}

    Vec2(const Vec2 &c) = default;

    bool operator==(const Vec2 &rhs) const { return x == rhs.x && y == rhs.y; };

    Vec2 operator+(const Vec2 &rhs) const { return Vec2(x + rhs.x, y + rhs.y); }

    Vec2 operator-(const Vec2 &rhs) const { return Vec2(x - rhs.x, y - rhs.y); }

    Vec2 operator*(const float t) const { return Vec2(x * t, y * t); }

    void operator*=(const float t) {
        x *= t;
        y *= t;
    }

    void operator+=(const Vec2 &rhs) {
        x += rhs.x;
        y += rhs.y;
    }

    Vec2 normalize() const {
        float abs = norm();
        if (abs < 1e-6f) {
            return Vec2(x, y);
        } else {
            Vec2 scaled_copy(x, y);
            scaled_copy *= (1.0 / abs);
            return scaled_copy;
        }
    }

    float dot(const Vec2 &rhs) const { return x * rhs.x + y * rhs.y; }

    float norm() const { return sqrt(x * x + y * y); }

    friend std::ostream &operator<<(std::ostream &, const Vec2 &);
};

std::ostream &operator<<(std::ostream &out, const Vec2 &v) {
    out << "Vec2(" << v.x << ", " << v.y << ")";
    return out;
}
