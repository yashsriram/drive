#pragma once

#include <eigen3/Eigen/Dense>

#include "vec2.hpp"

struct Line {
    const Vec2 p1;
    const Vec2 p2;

    Line(const Vec2& p1, const Vec2& p2) : p1(p1), p2(p2) {}

    bool does_intersect(const Vec2& end1, const Vec2& end2) const {
        // A
        Eigen::Matrix2f A;
        A << end2.y - end1.y, -(end2.x - end1.x), p2.y - p1.y, -(p2.x - p1.x);
        // b
        Eigen::Vector2f b;
        b << end1.x * end2.y - end1.y * end2.x, p1.x * p2.y - p1.y * p2.x;
        if (abs(A.determinant()) < 1e-6) {
            Vec2 e1 = end2 - p1;
            Vec2 e2 = p2 - end1;
            // Parallel or Coincident, we will just assume not intersecting for simplicity
            return false;
        }
        // Ax = b
        Eigen::Vector2f x2f = A.inverse() * b;
        Vec2 x(x2f[0], x2f[1]);

        // Is intersection b/w edge end points?
        float edge_length = (end2 - end1).norm();
        Vec2 edge_unit = (end2 - end1).normalize();
        Vec2 t_times_edge_unit = x - end1;

        float t1 = 0;
        int den = 0;
        if (abs(edge_unit.x) > 1e-4) {
            t1 += t_times_edge_unit.x / edge_unit.x;
            den++;
        }
        if (abs(edge_unit.y) > 1e-4) {
            t1 += t_times_edge_unit.y / edge_unit.y;
            den++;
        }
        t1 = t1 / den;
        if (t1 < 0 || t1 > edge_length) {
            return false;
        }

        // Is intersection b/w line end points?
        float line_length = (p2 - p1).norm();
        Vec2 line_unit = (p2 - p1).normalize();
        Vec2 t_times_line_unit = x - p1;
        float t2 = 0;
        den = 0;
        if (abs(line_unit.x) > 1e-4) {
            t2 += t_times_line_unit.x / line_unit.x;
            den++;
        }
        if (abs(line_unit.y) > 1e-4) {
            t2 += t_times_line_unit.y / line_unit.y;
            den++;
        }
        t2 = t2 / den;
        if (t2 < 0 || t2 > line_length) {
            return false;
        }

        return true;
    }
};
