#include <queue>
#include <vector>

#include "plan/visibility/vertex.hpp"
#include "sense/cs.hpp"

struct Visibility {
private:
    void add_to_fringe(std::queue<int>& fringe, const VisibilityVertex& current, VisibilityVertex& next) {
        next.distance_from_start = current.distance_from_start + (next.position - current.position).norm();
        next.is_explored = true;
        next.path_from_start = current.path_from_start;
        next.path_from_start.push_back(next.id);
        next.color = Eigen::Vector3f(0, 1, 0);
        fringe.push(next.id);
    }

    int add_vertex(const Vec2& position, const ConfigurationSpace& cs) {
        // Get the id of new vertex
        int id = vertices.size();
        // Add vertex
        vertices.push_back(VisibilityVertex(id, position.x, position.y));
        VisibilityVertex& new_vertex = vertices[vertices.size() - 1];
        // Add edges to neighbours properly
        for (int i = 0; i < vertices.size() - 1; ++i) {
            VisibilityVertex& neighbour = vertices[i];
            // Intersects with obstacles
            bool does_intersect = cs.does_intersect(new_vertex.position, neighbour.position);
            if (does_intersect) {
                continue;
            }
            // Add edge if all okay
            new_vertex.add_neighbour(neighbour.id);
            neighbour.add_neighbour(new_vertex.id);
        }
        return id;
    }

public:
    std::vector<VisibilityVertex> vertices;

    Visibility(const Vec2& start, const Vec2& finish, const ConfigurationSpace& cs) {
        add_vertex(start, cs);
        add_vertex(finish, cs);
        for (const Rectangle& rect : cs.get_rectangles()) {
            const float w = rect.width + 0.2;
            const float h = rect.height + 0.2;
            int v1 = add_vertex(rect.center + Vec2(-w / 2, h / 2).rotate(rect.orientation), cs);
            int v2 = add_vertex(rect.center + Vec2(w / 2, h / 2).rotate(rect.orientation), cs);
            int v3 = add_vertex(rect.center + Vec2(w / 2, -h / 2).rotate(rect.orientation), cs);
            int v4 = add_vertex(rect.center + Vec2(-w / 2, -h / 2).rotate(rect.orientation), cs);
            vertices[v1].add_neighbour(v2);
            vertices[v2].add_neighbour(v1);
            vertices[v2].add_neighbour(v3);
            vertices[v3].add_neighbour(v2);
            vertices[v3].add_neighbour(v4);
            vertices[v4].add_neighbour(v3);
            vertices[v4].add_neighbour(v1);
            vertices[v1].add_neighbour(v4);
        }
    }

    std::vector<Vec2> bfs(const ConfigurationSpace& cs) {
        // Reset search state of all vertices
        for (VisibilityVertex& vertex : vertices) {
            vertex.reset_search_state();
        }

        int num_vertices_explored = 0;

        std::queue<int> fringe;
        // Add start to fringe
        const int start_id = 0;
        const int finish_id = 1;
        int nearest_to_finish_vertex_id = start_id;
        float nearest_to_finish_distance = (vertices[start_id].position - vertices[finish_id].position).norm();
        add_to_fringe(fringe, vertices[start_id], vertices[start_id]);
        while (fringe.size() > 0) {
            // Pop one vertex
            int current_id = fringe.front();
            VisibilityVertex& current = vertices[current_id];
            fringe.pop();
            num_vertices_explored++;
            // Check if finish
            float distance_to_finish = (current.position - vertices[finish_id].position).norm();
            if (distance_to_finish < nearest_to_finish_distance) {
                nearest_to_finish_distance = distance_to_finish;
                nearest_to_finish_vertex_id = current_id;
            }
            // Mark this vertex as explored
            current.color = Eigen::Vector3f(1, 0, 0);
            // Update fringe
            for (int neighbourId : current.neighbour_ids) {
                VisibilityVertex& neighbour = vertices[neighbourId];
                if (!neighbour.is_explored) {
                    add_to_fringe(fringe, current, neighbour);
                }
            }
        }

        std::vector<Vec2> path;
        for (int id : vertices[nearest_to_finish_vertex_id].path_from_start) {
            path.push_back(vertices[id].position);
        }
        return path;
    }

    void draw(const ros::Publisher& viz) {
        visualization_msgs::MarkerArray arr;

        visualization_msgs::Marker vertex_marker;
        vertex_marker.header.frame_id = "map";
        vertex_marker.ns = "vertices";
        vertex_marker.header.stamp = ros::Time::now();
        vertex_marker.id = 0;
        vertex_marker.action = visualization_msgs::Marker::ADD;
        vertex_marker.pose.orientation.w = 1.0;
        vertex_marker.type = visualization_msgs::Marker::POINTS;
        vertex_marker.scale.x = 0.05;
        vertex_marker.scale.y = 0.05;

        // Vertices
        for (const auto& vertex : vertices) {
            geometry_msgs::Point p;
            p.x = vertex.position.x;
            p.y = vertex.position.y;
            vertex_marker.points.push_back(p);
            std_msgs::ColorRGBA c;
            c.a = 1.0;
            c.r = vertex.color[0];
            c.g = vertex.color[1];
            c.b = vertex.color[2];
            vertex_marker.colors.push_back(c);
        }

        arr.markers.push_back(vertex_marker);

        visualization_msgs::Marker edge_marker;
        edge_marker.header.frame_id = "map";
        edge_marker.ns = "edges";
        edge_marker.header.stamp = ros::Time::now();
        edge_marker.id = 0;
        edge_marker.action = visualization_msgs::Marker::ADD;
        edge_marker.pose.orientation.w = 1.0;
        edge_marker.color.a = 1.0;
        edge_marker.type = visualization_msgs::Marker::LINE_LIST;
        edge_marker.scale.x = 0.01;
        edge_marker.color.r = edge_marker.color.g = edge_marker.color.b = 1.0f;

        // Edges
        for (const auto& vertex : vertices) {
            for (auto neighbourId : vertex.neighbour_ids) {
                geometry_msgs::Point p1;
                p1.x = vertex.position.x;
                p1.y = vertex.position.y;
                edge_marker.points.push_back(p1);
                geometry_msgs::Point p2;
                p2.x = vertices[neighbourId].position.x;
                p2.y = vertices[neighbourId].position.y;
                edge_marker.points.push_back(p2);
            }
        }

        arr.markers.push_back(edge_marker);

        viz.publish(arr);
    }
};
