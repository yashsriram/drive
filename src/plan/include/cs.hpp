#pragma once

#include <math.h>
#include <visualization_msgs/MarkerArray.h>

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

    void draw(const ros::Publisher& viz) {
        // Draw circles
        visualization_msgs::MarkerArray arr;
        for (int i = 0; i < circles.size(); ++i) {
            const Circle& circle = circles[i];
            visualization_msgs::Marker m;

            m.header.frame_id = "map";
            m.ns = "circles";
            m.id = i;
            m.header.stamp = ros::Time::now();

            m.action = visualization_msgs::Marker::ADD;

            m.type = visualization_msgs::Marker::CYLINDER;
            m.pose.orientation.w = 1.0;
            m.pose.position.x = circle.center.x;
            m.pose.position.y = circle.center.y;

            m.scale.x = circle.radius * 2;
            m.scale.y = circle.radius * 2;
            m.scale.z = 0.1;

            m.color.r = 1;
            m.color.g = 1;
            m.color.b = 1;
            m.color.a = 0.2;

            arr.markers.push_back(m);
        }
        for (int i = 0; i < circles.size(); ++i) {
            const Circle& circle = circles[i];
            visualization_msgs::Marker m;

            m.header.frame_id = "map";
            m.ns = "cs_circles";
            m.id = i;
            m.header.stamp = ros::Time::now();

            m.action = visualization_msgs::Marker::ADD;

            m.type = visualization_msgs::Marker::CYLINDER;
            m.pose.orientation.w = 1.0;
            m.pose.position.x = circle.center.x;
            m.pose.position.y = circle.center.y;

            m.scale.x = (circle.radius + agent_radius) * 2;
            m.scale.y = (circle.radius + agent_radius) * 2;
            m.scale.z = 0.1;

            m.color.r = 1;
            m.color.g = 0;
            m.color.b = 0;
            m.color.a = 0.2;

            arr.markers.push_back(m);
        }
        viz.publish(arr);
    }
};

