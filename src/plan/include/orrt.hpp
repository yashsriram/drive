#pragma once

#include <random>
#include <stack>
#include <unordered_map>
#include <vector>

#include "cs.hpp"
#include "graph.hpp"
#include "vec2.hpp"

struct ORRT {
    /* int get_finish_vertex() { */
    /*     std::stack<const Vertex *> fringe; */
    /*     fringe.push(&root); */
    /*     Vertex *finishVertex = NULL; */
    /*     while (fringe.size() > 0) { */
    /*         const Vertex *node = fringe.top(); */
    /*         fringe.pop(); */
    /*         if ((node->position - finish_position).norm() < 1e-6) { */
    /*             finishVertex = (Vertex *)node; */
    /*             break; */
    /*         } */
    /*         for (const Vertex &child : node->children) { */
    /*             fringe.push(&child); */
    /*         } */
    /*     } */
    /*     return finishVertex; */
    /* } */

    constexpr static float GROWTH_LIMIT = 1;
    constexpr static float END_POINT_HINT_SIZE = 0.2;
    constexpr static float NEIGHBOUR_RADIUS = 1;
    constexpr static bool DRAW_TREE = true;

    const Vec2 start_position;
    const Vec2 finish_position;
    bool is_finish_reached;
    Graph graph;

    ORRT(const Vec2 &start_position, const Vec2 &finish_position) : start_position(start_position), finish_position(finish_position), graph(start_position, 0), is_finish_reached(false) {}

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
        if (new_position == finish_position) {
            is_finish_reached = true;
        }
    }

    void grow_tree(const std::vector<Vec2> &new_positions, const ConfigurationSpace &cs) {
        std::random_device rd;
        std::mt19937 e2(rd());
        std::uniform_real_distribution<> dist(0, 1);
        for (const Vec2 &new_position : new_positions) {
            // Generate node at finish position with a small probability
            if (dist(e2) <= 0.01) {
                generate_next_node(finish_position, cs);
            }
            generate_next_node(new_position, cs);
        }
    }

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

        viz.publish(arr);
    }

    /*     public List<Vec2> search() { */
    /*         Vertex finishVertex = get_finish_vertex(); */
    /*         if (finishVertex != null) { */
    /*             List<Vec2> path = new ArrayList<>(); */
    /*             path.add(0, finishVertex.position); */
    /*             Vertex node = finishVertex.parent; */
    /*             while (node != null) { */
    /*                 path.add(0, node.position); */
    /*                 node = node.parent; */
    /*             } */
    /*             return path; */
    /*         } */
    /*         PApplet.println("Could not find path to finish position"); */
    /*         return Collections.singletonList(start_position); */
    /*     } */
};

