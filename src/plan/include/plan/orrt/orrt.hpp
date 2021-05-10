#pragma once

#include <random>
#include <stack>
#include <unordered_map>
#include <vector>

#include "plan/orrt/graph.hpp"
#include "sense/cs.hpp"
#include "vec2.hpp"

struct ORRT {
    constexpr static float GROWTH_LIMIT = 0.5;
    constexpr static float END_POINT_HINT_SIZE = 0.2;
    constexpr static float NEIGHBOUR_RADIUS = 0.5;
    constexpr static bool DRAW_TREE = true;

    const Vec2 start_position;
    const Vec2 finish_position;
    const float sampling_padding;
    Graph graph;
    int nearest_vertex_to_finish_id;
    float nearest_vertex_to_finish_distance;

    ORRT(const Vec2 &start_position, const Vec2 &finish_position, const float sampling_padding)
        : start_position(start_position),
          finish_position(finish_position),
          graph(start_position, 0),
          sampling_padding(sampling_padding),
          nearest_vertex_to_finish_id(graph.root().id),
          nearest_vertex_to_finish_distance((start_position - finish_position).norm()) {
        if (sampling_padding == 0) {
            throw std::runtime_error("Zero sampling padding.");
        }
    }

    void generate_next_node(Vec2 new_position, const ConfigurationSpace &cs) {
        // Nearest vertex search
        std::stack<int> fringe;
        std::vector<int> neighbours_ids;
        fringe.push(graph.root_id);
        float min_distance = (new_position - graph.root().position).norm();
        int nearest_vertex_id = graph.root_id;
        while (fringe.size() > 0) {
            const int node_id = fringe.top();
            fringe.pop();
            const Vertex &node = graph.get(node_id);
            if ((node.position - new_position).norm() < NEIGHBOUR_RADIUS) {
                neighbours_ids.push_back(node_id);
            }
            float distance = (new_position - node.position).norm();
            if (distance < min_distance) {
                min_distance = distance;
                nearest_vertex_id = node_id;
            }
            for (int child_id : node.children_ids) {
                fringe.push(child_id);
            }
        }
        // If no neighbours_ids exist within given range at least we will have the
        // Nearest neighbour
        if (neighbours_ids.size() == 0) {
            neighbours_ids.push_back(nearest_vertex_id);
        }
        // Min cost vertex search
        int min_cost_vertex_id = nearest_vertex_id;
        const Vertex &nearest_vertex = graph.get(nearest_vertex_id);
        float min_cost = nearest_vertex.cost_from_start + (nearest_vertex.position - new_position).norm();
        for (int neighbour_id : neighbours_ids) {
            const Vertex &neighbour = graph.get(neighbour_id);
            float cost = neighbour.cost_from_start + (neighbour.position - new_position).norm();
            if (cost < min_cost) {
                min_cost_vertex_id = neighbour_id;
                min_cost = cost;
            }
        }
        // Growth limit
        const Vertex &min_cost_vertex = graph.get(min_cost_vertex_id);
        Vec2 growth = (new_position - min_cost_vertex.position);
        if (growth.norm() > GROWTH_LIMIT) {
            new_position = (min_cost_vertex.position + growth.normalize() * GROWTH_LIMIT);
        }
        // Collision detection
        if (cs.does_intersect(min_cost_vertex.position, new_position)) {
            return;
        }
        // Linking min cost vertex and new vertex
        float distance_from_start = min_cost_vertex.cost_from_start + (min_cost_vertex.position - new_position).norm();
        int new_vertex_id = graph.add_child(min_cost_vertex_id, new_position, distance_from_start);
        const Vertex &new_vertex = graph.get(new_vertex_id);
        // Rewiring
        for (int neighbour_id : neighbours_ids) {
            Vertex &neighbour = graph.get_mut(neighbour_id);
            float cost = distance_from_start + (new_position - neighbour.position).norm();
            if (cost < neighbour.cost_from_start) {
                if (!cs.does_intersect(neighbour.position, new_vertex.position)) {
                    neighbour.cost_from_start = cost;
                    graph.reparent(neighbour.id, new_vertex_id);
                }
            }
        }
        float distance_to_finish = (new_vertex.position - finish_position).norm();
        if (distance_to_finish < nearest_vertex_to_finish_distance) {
            nearest_vertex_to_finish_id = new_vertex_id;
            nearest_vertex_to_finish_distance = distance_to_finish;
        }
    }

    void grow_tree(int num_nodes, const ConfigurationSpace &cs) {
        std::random_device rd;
        std::mt19937 e2(rd());
        std::uniform_real_distribution<> dist(0, 1);
        const float len = (finish_position - start_position).norm();
        const Vec2 unit = (finish_position - start_position).normalize();
        for (int i = 0; i < num_nodes; ++i) {
            // Generate node at finish position with a small probability
            if (dist(e2) <= 0.01) {
                generate_next_node(finish_position, cs);
            } else {
                float x = dist(e2) * len;
                float y = dist(e2) * sampling_padding * 2 - sampling_padding;
                Vec2 rotated_sample_position(x * unit.x - y * unit.y, x * unit.y + y * unit.x);
                Vec2 rot_trans_sample_position = rotated_sample_position + start_position;
                generate_next_node(rot_trans_sample_position, cs);
            }
        }
    }

    std::vector<Vec2> path_to_nearest_node_from_finish() { return graph.get_path_to(nearest_vertex_to_finish_id); }

    void print() {
        std::stack<int> fringe;
        fringe.push(graph.root_id);
        while (fringe.size() > 0) {
            int node_id = fringe.top();
            fringe.pop();
            const Vertex &node = graph.get(node_id);

            for (int i = 0; i < node.children_ids.size(); ++i) {
                const Vertex &child = graph.get(node.children_ids[i]);
            }

            for (int child_id : node.children_ids) {
                fringe.push(child_id);
            }
        }
    }

    void draw(const ros::Publisher &viz) {
        // Viz start and end
        visualization_msgs::MarkerArray arr;

        visualization_msgs::Marker s;
        s.header.frame_id = "map";
        s.ns = "ends";
        s.id = 1;
        s.header.stamp = ros::Time::now();
        s.action = visualization_msgs::Marker::ADD;
        s.type = visualization_msgs::Marker::CUBE;
        s.scale.x = END_POINT_HINT_SIZE;
        s.scale.y = END_POINT_HINT_SIZE;
        s.scale.z = END_POINT_HINT_SIZE;
        s.pose.position.x = start_position.x;
        s.pose.position.y = start_position.y;
        s.pose.position.z = 0;
        s.pose.orientation.w = 1.0;
        s.color.b = 1;
        s.color.a = 1.0;
        arr.markers.push_back(s);

        visualization_msgs::Marker f;
        f.header.frame_id = "map";
        f.ns = "ends";
        f.id = 2;
        f.header.stamp = ros::Time::now();
        f.action = visualization_msgs::Marker::ADD;
        f.type = visualization_msgs::Marker::CUBE;
        f.scale.x = END_POINT_HINT_SIZE;
        f.scale.y = END_POINT_HINT_SIZE;
        f.scale.z = END_POINT_HINT_SIZE;
        f.pose.position.x = finish_position.x;
        f.pose.position.y = finish_position.y;
        f.pose.position.z = 0;
        f.pose.orientation.w = 1.0;
        f.color.g = 1;
        f.color.a = 1.0;
        arr.markers.push_back(f);

        // Viz sampling region
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.ns = "sampling_region";
        m.id = 0;
        m.header.stamp = ros::Time::now();
        m.action = visualization_msgs::Marker::ADD;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.pose.orientation.w = 1;
        const Vec2 unit = (finish_position - start_position).normalize();
        const Vec2 ccw_perpendicular(-unit.y, unit.x), cw_perpendicular(unit.y, -unit.x);
        const Vec2 e1 = start_position + ccw_perpendicular * sampling_padding;
        const Vec2 e2 = finish_position + ccw_perpendicular * sampling_padding;
        const Vec2 e3 = finish_position + cw_perpendicular * sampling_padding;
        const Vec2 e4 = start_position + cw_perpendicular * sampling_padding;
        geometry_msgs::Point p1;
        p1.x = e1.x;
        p1.y = e1.y;
        m.points.push_back(p1);
        geometry_msgs::Point p2;
        p2.x = e2.x;
        p2.y = e2.y;
        m.points.push_back(p2);
        geometry_msgs::Point p3;
        p3.x = e3.x;
        p3.y = e3.y;
        m.points.push_back(p3);
        geometry_msgs::Point p4;
        p4.x = e4.x;
        p4.y = e4.y;
        m.points.push_back(p4);
        geometry_msgs::Point p5;
        p5.x = e1.x;
        p5.y = e1.y;
        m.points.push_back(p5);
        m.scale.x = 0.05;
        m.color.r = 0;
        m.color.g = 1;
        m.color.b = 1;
        m.color.a = 0.5;
        arr.markers.push_back(m);

        // Viz tree
        if (DRAW_TREE) {
            std::stack<int> fringe;
            fringe.push(graph.root_id);
            int edge_id = 0;
            visualization_msgs::MarkerArray arr;
            while (fringe.size() > 0) {
                int node_id = fringe.top();
                fringe.pop();
                const Vertex &node = graph.get(node_id);

                for (int i = 0; i < node.children_ids.size(); ++i) {
                    const Vertex &child = graph.get(node.children_ids[i]);
                    visualization_msgs::Marker m;

                    m.header.frame_id = "map";
                    m.ns = "edge";
                    m.id = edge_id;
                    edge_id++;
                    m.header.stamp = ros::Time::now();

                    m.action = visualization_msgs::Marker::ADD;
                    m.type = visualization_msgs::Marker::ARROW;

                    geometry_msgs::Point start;
                    start.x = node.position.x;
                    start.y = node.position.y;
                    start.z = 0;
                    m.points.push_back(start);
                    geometry_msgs::Point end;
                    end.x = child.position.x;
                    end.y = child.position.y;
                    end.z = 0;
                    m.points.push_back(end);

                    m.scale.x = 0.01;
                    m.scale.y = 0.03;
                    m.scale.z = 0;
                    m.pose.orientation.w = 1.0;

                    m.color.r = 1;
                    m.color.g = 1;
                    m.color.b = 1;
                    m.color.a = 1.0;

                    arr.markers.push_back(m);
                }

                for (int child_id : node.children_ids) {
                    fringe.push(child_id);
                }
            }
            viz.publish(arr);
        }

        viz.publish(arr);
    }
};

