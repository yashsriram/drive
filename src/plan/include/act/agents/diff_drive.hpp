#pragma once

#include <visualization_msgs/MarkerArray.h>

#include "sense/cs.hpp"
#include "vec2.hpp"

struct DiffDrive {
    const float MILESTONE_SLACK = 0.04f;
    const float ORIENTATION_SLACK = 0.01f;

    Vec2 center;
    float orientation;
    float radius;

    float linear_speed;
    float angular_speed;

    std::vector<Vec2> path;
    int current_milestone = 0;

    DiffDrive(const Vec2& center, const float orientation, const float radius, const float linear_speed, const float angular_speed)
        : center(center), orientation(orientation), radius(radius), linear_speed(linear_speed), angular_speed(angular_speed) {
        path.push_back(center);
        path.push_back(Vec2(0, 7));
        path.push_back(Vec2(4, 4));
    }

    /* void set_path(std::vector<Vec2> new_path) { path = new_path; } */

    void update(float dt, const ConfigurationSpace& cs) {
        if (current_milestone < path.size() - 1) {
            // Pull towards next milestone
            Vec2 to_goal = path[current_milestone + 1] - center;
            Vec2 velocity = to_goal.normalize() * linear_speed;
            Vec2 displacement = velocity * dt;

            float goal_orientation = atan2(displacement.y, displacement.x);
            float to_orientation = goal_orientation - orientation;
            // Orient towards goal
            if (abs(to_orientation) > ORIENTATION_SLACK) {
                orientation += (to_orientation > 0) ? angular_speed * dt : -angular_speed * dt;
                return;
            }
            // Reached next milestone
            if (to_goal.norm() < MILESTONE_SLACK) {
                current_milestone++;
                return;
            }
            // Next next milestone lookup
            if (current_milestone < path.size() - 2) {
                bool blocked = cs.does_intersect(center, path[current_milestone + 2]);
                if (!blocked) {
                    current_milestone++;
                }
            }

            // Move towards next milestone
            center += displacement;
        }
    }

    void draw(const ros::Publisher& viz) {
        visualization_msgs::MarkerArray arr;

        // Position
        visualization_msgs::Marker center_marker;
        center_marker.header.frame_id = "map";
        center_marker.ns = "diff_drive";
        center_marker.header.stamp = ros::Time::now();
        center_marker.id = 0;
        center_marker.type = visualization_msgs::Marker::CYLINDER;
        center_marker.action = visualization_msgs::Marker::ADD;
        center_marker.pose.position.x = center.x;
        center_marker.pose.position.y = center.y;
        center_marker.pose.orientation.w = 1.0;
        center_marker.scale.x = 2 * radius;
        center_marker.scale.y = 2 * radius;
        center_marker.scale.z = 0.1;
        center_marker.color.r = 1.0;
        center_marker.color.g = 1.0;
        center_marker.color.b = 1.0;
        center_marker.color.a = 1.0;
        arr.markers.push_back(center_marker);

        // Orientation
        visualization_msgs::Marker orientation_marker;
        orientation_marker.header.frame_id = "map";
        orientation_marker.ns = "diff_drive";
        orientation_marker.header.stamp = ros::Time::now();
        orientation_marker.id = 1;
        orientation_marker.action = visualization_msgs::Marker::ADD;
        orientation_marker.pose.orientation.w = 1.0;
        orientation_marker.color.a = 1.0;
        orientation_marker.type = visualization_msgs::Marker::LINE_LIST;
        orientation_marker.scale.x = 0.01;
        orientation_marker.color.g = 1.0f;
        orientation_marker.color.b = 1.0f;
        geometry_msgs::Point p1;
        p1.x = center.x;
        p1.y = center.y;
        p1.z = 0.15;
        orientation_marker.points.push_back(p1);
        geometry_msgs::Point p2;
        p2.x = center.x + cos(orientation) * radius;
        p2.y = center.y + sin(orientation) * radius;
        p2.z = 0.15;
        orientation_marker.points.push_back(p2);
        arr.markers.push_back(orientation_marker);

        // Next center
        if (current_milestone < path.size() - 1) {
            Vec2 next_center = path[current_milestone + 1];
            visualization_msgs::Marker next_center_marker;
            next_center_marker.header.frame_id = "map";
            next_center_marker.ns = "diff_drive";
            next_center_marker.header.stamp = ros::Time::now();
            next_center_marker.id = 2;
            next_center_marker.type = visualization_msgs::Marker::CYLINDER;
            next_center_marker.action = visualization_msgs::Marker::ADD;
            next_center_marker.pose.position.x = next_center.x;
            next_center_marker.pose.position.y = next_center.y;
            next_center_marker.pose.orientation.w = 1.0;
            next_center_marker.scale.x = 2 * radius;
            next_center_marker.scale.y = 2 * radius;
            next_center_marker.scale.z = 0.01;
            next_center_marker.color.r = 1.0f;
            next_center_marker.color.a = 1.0;
            arr.markers.push_back(next_center_marker);
        }

        // Path
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.ns = "diff_drive_path";
        path_marker.header.stamp = ros::Time::now();
        path_marker.id = 0;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.pose.orientation.w = 1.0;
        path_marker.color.a = 1.0;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.scale.x = 0.01;
        path_marker.color.r = path_marker.color.g = path_marker.color.b = 1.0f;

        // Links
        for (const auto& milestone : path) {
            geometry_msgs::Point p;
            p.x = milestone.x;
            p.y = milestone.y;
            path_marker.points.push_back(p);
        }

        arr.markers.push_back(path_marker);

        viz.publish(arr);
    }
};

