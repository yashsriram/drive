#pragma once

#include <math.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Dense>

#include "route.hpp"
#include "sense/obstacles/line.hpp"
#include "sense/obstacles/rectangle.hpp"

struct ConfigurationSpace {
private:
    std::vector<Rectangle> rectangles;
    std::vector<Rectangle> physical_rectangles;
    std::vector<Line> lines;
    std::vector<Line> physical_lines;

    void add_lines(const std::vector<Line>& side_lines) {
        if (side_lines.size() == 0) {
            throw std::runtime_error("Found zero side_lines.");
        }
        Vec2 start = side_lines[0].p1;
        for (int i = 1; i < side_lines.size(); ++i) {
            const Line& prev_line = side_lines[i - 1];
            const Line& curr_line = side_lines[i];
            const Vec2 end1 = prev_line.p1;
            const Vec2 end2 = prev_line.p2;
            const Vec2 p1 = curr_line.p1;
            const Vec2 p2 = curr_line.p2;
            // A
            Eigen::Matrix2f A;
            A << end2.y - end1.y, -(end2.x - end1.x), p2.y - p1.y, -(p2.x - p1.x);
            // b
            Eigen::Vector2f b;
            b << end1.x * end2.y - end1.y * end2.x, p1.x * p2.y - p1.y * p2.x;
            if (abs(A.determinant()) < 1e-6) {
                if ((prev_line.p2 - curr_line.p1).norm() < 1e-6) {
                    // Coincident
                    lines.push_back(Line(start, curr_line.p1));
                    start = curr_line.p1;
                } else {
                    // Parallel
                    throw std::runtime_error("Found consecutive coincident route segments.");
                }
            } else {
                // Ax = b
                Eigen::Vector2f x2f = A.inverse() * b;
                Vec2 x(x2f[0], x2f[1]);
                lines.push_back(Line(start, x));
                start = x;
            }
        }
        Vec2 end = side_lines[side_lines.size() - 1].p2;
        lines.push_back(Line(start, end));
    }

public:
    ConfigurationSpace() {}

    const std::vector<Rectangle>& get_rectangles() const { return rectangles; }

    void clear_rectangles() {
        rectangles.clear();
        physical_rectangles.clear();
    }

    void add_rectangle(const float& x, const float& y, const float& orientation, const float& width, const float& height, const float& agent_radius) {
        physical_rectangles.push_back(Rectangle(Vec2(x, y), orientation, width, height));
        rectangles.push_back(Rectangle(Vec2(x, y), orientation, width + agent_radius * 2, height + agent_radius * 2));
    }

    // Does not handle near 180 turns, intersecting/looped routes
    void add_lines(const Route& route, const float& padding, const float& agent_radius) {
        if (padding <= agent_radius) {
            throw std::runtime_error("Route padding <= Agent radius.");
        }
        const std::vector<Vec2>& points = route.points;
        float cs_padding = padding - agent_radius;
        std::vector<Line> left_lines;
        std::vector<Line> right_lines;
        for (int i = 1; i < points.size(); ++i) {
            const Vec2& p1 = points[i - 1];
            const Vec2& p2 = points[i];
            const Vec2 unit = (p2 - p1).normalize();
            const Vec2 ccw_perpendicular(-unit.y, unit.x), cw_perpendicular(unit.y, -unit.x);
            left_lines.push_back(Line(p1 + ccw_perpendicular * cs_padding, p2 + ccw_perpendicular * cs_padding));
            right_lines.push_back(Line(p1 + cw_perpendicular * cs_padding, p2 + cw_perpendicular * cs_padding));
            physical_lines.push_back(Line(p1, p2));
        }
        add_lines(left_lines);
        add_lines(right_lines);
    }

    bool does_intersect(const Vec2& p1, const Vec2& p2) const {
        for (const auto& rectangle : rectangles) {
            if (rectangle.does_intersect(p1, p2)) {
                return true;
            }
        }
        for (const auto& line : lines) {
            if (line.does_intersect(p1, p2)) {
                return true;
            }
        }
        return false;
    }

    void draw(const ros::Publisher& viz) const {
        // Draw rectangles
        visualization_msgs::MarkerArray arr;
        for (int i = 0; i < rectangles.size(); ++i) {
            const Rectangle& rectangle = rectangles[i];
            visualization_msgs::Marker m;

            m.header.frame_id = "map";
            m.ns = "rectangles";
            m.id = i;
            m.header.stamp = ros::Time::now();

            m.action = visualization_msgs::Marker::ADD;

            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.pose.orientation.w = 1;
            geometry_msgs::Point p1;
            p1.x = rectangle.l1.p1.x;
            p1.y = rectangle.l1.p1.y;
            m.points.push_back(p1);
            geometry_msgs::Point p2;
            p2.x = rectangle.l2.p1.x;
            p2.y = rectangle.l2.p1.y;
            m.points.push_back(p2);
            geometry_msgs::Point p3;
            p3.x = rectangle.l3.p1.x;
            p3.y = rectangle.l3.p1.y;
            m.points.push_back(p3);
            geometry_msgs::Point p4;
            p4.x = rectangle.l4.p1.x;
            p4.y = rectangle.l4.p1.y;
            m.points.push_back(p4);
            geometry_msgs::Point p5;
            p5.x = rectangle.l1.p1.x;
            p5.y = rectangle.l1.p1.y;
            m.points.push_back(p5);

            m.scale.x = 0.05;

            m.color.r = 1;
            m.color.g = 0;
            m.color.b = 1;
            m.color.a = 0.5;

            arr.markers.push_back(m);
        }
        // Draw physical rectangles
        for (int i = 0; i < physical_rectangles.size(); ++i) {
            const Rectangle& rectangle = physical_rectangles[i];
            visualization_msgs::Marker m;

            m.header.frame_id = "map";
            m.ns = "physical_rectangles";
            m.id = i;
            m.header.stamp = ros::Time::now();

            m.action = visualization_msgs::Marker::ADD;

            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.pose.orientation.w = 1;
            geometry_msgs::Point p1;
            p1.x = rectangle.l1.p1.x;
            p1.y = rectangle.l1.p1.y;
            m.points.push_back(p1);
            geometry_msgs::Point p2;
            p2.x = rectangle.l2.p1.x;
            p2.y = rectangle.l2.p1.y;
            m.points.push_back(p2);
            geometry_msgs::Point p3;
            p3.x = rectangle.l3.p1.x;
            p3.y = rectangle.l3.p1.y;
            m.points.push_back(p3);
            geometry_msgs::Point p4;
            p4.x = rectangle.l4.p1.x;
            p4.y = rectangle.l4.p1.y;
            m.points.push_back(p4);
            geometry_msgs::Point p5;
            p5.x = rectangle.l1.p1.x;
            p5.y = rectangle.l1.p1.y;
            m.points.push_back(p5);

            m.scale.x = 0.05;

            m.color.r = 1;
            m.color.g = 0;
            m.color.b = 1;
            m.color.a = 0.5;

            arr.markers.push_back(m);
        }
        // Draw lines
        for (int i = 0; i < lines.size(); ++i) {
            const Line& line = lines[i];
            visualization_msgs::Marker m;

            m.header.frame_id = "map";
            m.ns = "lines";
            m.id = i;
            m.header.stamp = ros::Time::now();

            m.action = visualization_msgs::Marker::ADD;

            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.pose.orientation.w = 1;
            geometry_msgs::Point p1;
            p1.x = line.p1.x;
            p1.y = line.p1.y;
            m.points.push_back(p1);
            geometry_msgs::Point p2;
            p2.x = line.p2.x;
            p2.y = line.p2.y;
            m.points.push_back(p2);

            m.scale.x = 0.05;

            m.color.r = 1;
            m.color.g = 0;
            m.color.b = 1;
            m.color.a = 0.5;

            arr.markers.push_back(m);
        }
        // Draw physical lines
        for (int i = 0; i < physical_lines.size(); ++i) {
            const Line& line = physical_lines[i];
            visualization_msgs::Marker m;

            m.header.frame_id = "map";
            m.ns = "physical_lines";
            m.id = i;
            m.header.stamp = ros::Time::now();

            m.action = visualization_msgs::Marker::ADD;

            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.pose.orientation.w = 1;
            geometry_msgs::Point p1;
            p1.x = line.p1.x;
            p1.y = line.p1.y;
            m.points.push_back(p1);
            geometry_msgs::Point p2;
            p2.x = line.p2.x;
            p2.y = line.p2.y;
            m.points.push_back(p2);

            m.scale.x = 0.05;

            m.color.r = 1;
            m.color.g = 1;
            m.color.b = 0;
            m.color.a = 0.5;

            arr.markers.push_back(m);
        }
        viz.publish(arr);
    }
};

