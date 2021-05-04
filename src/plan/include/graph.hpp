#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

#include "vec2.hpp"

namespace vertex_id_generator {
const int null_index = -1;
int next_id = 0;
int get_next_id() {
    int curr_id = next_id;
    next_id++;
    return curr_id;
}
}  // namespace vertex_id_generator

struct Vertex {
    int id;
    Vec2 position;
    float cost_from_start;
    std::vector<int> children_ids;
    int parent_id;

    Vertex() : id(vertex_id_generator::null_index), position(Vec2(0, 0)), cost_from_start(0), parent_id(vertex_id_generator::null_index) {}

    Vertex(int id, const Vec2 &position, float cost_from_start) : id(id), position(position), cost_from_start(cost_from_start), parent_id(vertex_id_generator::null_index) {}
};

struct Graph {
    std::unordered_map<int, Vertex> vertices;
    const int root_id;

    Graph(const Vec2 &position, float cost_from_start) : root_id(vertex_id_generator::get_next_id()) { vertices[root_id] = Vertex(root_id, position, cost_from_start); }

    int add_child(int parent_id, const Vec2 &position, float cost_from_start) {
        int child_id = vertex_id_generator::get_next_id();
        vertices[child_id] = Vertex(child_id, position, cost_from_start);
        vertices[child_id].parent_id = parent_id;
        vertices[parent_id].children_ids.push_back(child_id);
        return child_id;
    }

    void reparent(int child_id, int new_parent_id) {
        int parent_id = vertices[child_id].parent_id;
        int index_to_remove = -1;
        for (int i = 0; i < vertices[parent_id].children_ids.size(); ++i) {
            if (vertices[parent_id].children_ids[i] == child_id) {
                index_to_remove = i;
                break;
            }
        }
        vertices[parent_id].children_ids.erase(vertices[parent_id].children_ids.begin() + index_to_remove);
        vertices[child_id].parent_id = new_parent_id;
        vertices[new_parent_id].children_ids.push_back(child_id);
    }

    Vertex &root() { return vertices[root_id]; }

    Vertex &get(int id) { return vertices[id]; }

    unsigned int size() { return vertices.size(); }
};
