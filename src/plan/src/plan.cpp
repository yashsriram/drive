#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <cstdio>
#include <ctime>

#include "act/agents/diff_drive.hpp"
#include "plan/orrt/orrt.hpp"
#include "plan/visibility/graph.hpp"
#include "route.hpp"
#include "sense/cs.hpp"
#include "sense/obstacles/rectangle.hpp"
#include "vec2.hpp"

struct ObstaclesPublisher {
    std::vector<Rectangle> rectangles;

    ObstaclesPublisher() {
        rectangles.push_back(Rectangle(Vec2(0.0, 3.0), 0.2, 0.4, 0.2));
        rectangles.push_back(Rectangle(Vec2(3.0, 5.0), 0.4, 0.4, 0.2));
        rectangles.push_back(Rectangle(Vec2(8.0, 1.0), 0.2, 0.4, 0.2));
        rectangles.push_back(Rectangle(Vec2(11.0, 2.0), 0.3, 0.4, 0.2));
        rectangles.push_back(Rectangle(Vec2(11.0, 5.0), 0.8, 0.4, 0.2));
        rectangles.push_back(Rectangle(Vec2(11.0, 4.0), 0.2, 0.4, 0.2));
        rectangles.push_back(Rectangle(Vec2(11.0, 3.0), 0.1, 0.4, 0.2));
        rectangles.push_back(Rectangle(Vec2(8.0, 2.0), 0.2, 0.4, 0.2));
        rectangles.push_back(Rectangle(Vec2(8.0, 5.0), 0.7, 0.4, 0.2));
        rectangles.push_back(Rectangle(Vec2(8.0, 4.0), 0.7, 0.4, 0.2));
        rectangles.push_back(Rectangle(Vec2(8.0, 3.0), 0.8, 0.4, 0.2));
    }

    std::vector<Rectangle> get_obstacles(const Vec2& position, const float range) const {
        std::vector<Rectangle> ans;
        for (const Rectangle& rectangle : rectangles) {
            if ((rectangle.center - position).norm() < range) {
                ans.push_back(rectangle);
            }
        }
        return ans;
    }
};

const float AGENT_RADIUS = 0.25;
const float ROUTE_PADDING = AGENT_RADIUS * 6;
const float SAMPLING_PADDING = ROUTE_PADDING * 1.2;
const float SENSING_RANGE = 1.5;
ConfigurationSpace cs;

int main(int argc, char** argv) {
    ros::init(argc, argv, "plan_node");

    ros::NodeHandle n;
    ros::Publisher plan_viz = n.advertise<visualization_msgs::MarkerArray>("/plan/debug", 30);
    ros::Publisher sense_viz = n.advertise<visualization_msgs::MarkerArray>("/sense/debug", 1);
    ros::Publisher act_viz = n.advertise<visualization_msgs::MarkerArray>("/act/debug", 1);

    // Input
    Route route({Vec2(0.0, 0.0), Vec2(0.0, 5.0), Vec2(5.0, 5.0), Vec2(7.0, 5.0), Vec2(9.0, 0.0), Vec2(12.0, 5.0), Vec2(12.0, 8.0), Vec2(18.0, 8.0)});
    DiffDrive agent(route.start(), 0.5, AGENT_RADIUS, 3, 3, SENSING_RANGE);

    ObstaclesPublisher obs_pub;
    cs.add_lines(route, ROUTE_PADDING, AGENT_RADIUS);

    ros::Rate loop_rate(30);
    std::clock_t start;
    double cumulative_duration;
    int loop_iter = 0;
    while (ros::ok()) {
        // Done?
        if (route.is_done()) {
            break;
        }

        // Sense
        cs.clear_rectangles();
        for (const Rectangle& sensed_rectangle : obs_pub.get_obstacles(agent.center, SENSING_RANGE)) {
            cs.add_rectangle(sensed_rectangle.center.x, sensed_rectangle.center.y, sensed_rectangle.orientation, sensed_rectangle.width, sensed_rectangle.height, AGENT_RADIUS);
        }

        // Plan
        start = std::clock();

        /* Visibility vis_graph(agent.center, route.current_goal(), cs); */
        /* agent.set_path(vis_graph.bfs(cs)); */

        ORRT orrt(agent.center, route.current_goal(), SAMPLING_PADDING);
        orrt.grow_tree(250, cs);
        agent.set_path(orrt.path_to_nearest_node_from_finish());

        cumulative_duration += (std::clock() - start) / (double)CLOCKS_PER_SEC;

        // Act
        for (int i = 0; i < 5; i++) {
            bool reached_finish = agent.update(0.01, cs);
            if (reached_finish) {
                route.increment_goal();
                break;
            }
        }

        // Draw
        cs.draw(sense_viz);
        orrt.draw(plan_viz);
        /* vis_graph.draw(plan_viz); */
        agent.draw(act_viz);

        // Sleep
        loop_rate.sleep();
        loop_iter++;
    }
    std::cout << "average duration: " << cumulative_duration / loop_iter << std::endl;

    return 0;
}
