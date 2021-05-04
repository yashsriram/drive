#pragma once

#include <cmath>
#include <ostream>

struct Vec3 {
    float x, y, z;

    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3(const Vec3 &c) = default;

    bool operator==(const Vec3 &rhs) const { return x == rhs.x && y == rhs.y && z == rhs.z; };

    Vec3 operator+(const Vec3 &rhs) const { return Vec3(x + rhs.x, y + rhs.y, z + rhs.z); }

    Vec3 operator-(const Vec3 &rhs) const { return Vec3(x - rhs.x, y - rhs.y, z - rhs.z); }

    Vec3 operator*(float t) const { return Vec3(x * t, y * t, z * t); }

    void operator*=(float t) {
        x *= t;
        y *= t;
        z *= t;
    }

    Vec3 normalize() const {
        float abs = norm();
        if (abs < 1e-6f) {
            return Vec3(x, y, z);
        } else {
            Vec3 scaled_copy(x, y, z);
            scaled_copy *= (1.0 / abs);
            return scaled_copy;
        }
    }

    float dot(Vec3 rhs) const { return x * rhs.x + y * rhs.y + z * rhs.z; }

    Vec3 cross(Vec3 rhs) const { return Vec3(y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x); }

    float norm() const { return sqrt(x * x + y * y + z * z); }

    friend std::ostream &operator<<(std::ostream &, const Vec3 &);
};

std::ostream &operator<<(std::ostream &out, const Vec3 &v) {
    out << "Vec3(" << v.x << ", " << v.y << ", " << v.z << ")";
    return out;
}
