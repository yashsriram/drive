#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "act/agents/diff_drive.hpp"
#include "plan/orrt.hpp"
#include "route.hpp"
#include "sense/cs.hpp"
#include "sense/obstacles/circle.hpp"
#include "vec2.hpp"

using namespace std;

const float AGENT_RADIUS = 0.25;
const float ROUTE_PADDING = AGENT_RADIUS * 4;
const float SAMPLING_PADDING = ROUTE_PADDING * 2;
ConfigurationSpace cs;

int main(int argc, char** argv) {
    ros::init(argc, argv, "plan_node");

    ros::NodeHandle n;
    ros::Publisher plan_viz = n.advertise<visualization_msgs::MarkerArray>("/plan/debug", 30);
    ros::Publisher sense_viz = n.advertise<visualization_msgs::MarkerArray>("/sense/debug", 1);
    ros::Publisher act_viz = n.advertise<visualization_msgs::MarkerArray>("/act/debug", 1);

    // Input
    Route route({Vec2(0.0, 0.0), Vec2(1.0, 5.0), Vec2(5.0, 5.0), Vec2(7.0, 5.0), Vec2(9.0, 0.0)});
    DiffDrive agent(route.start(), 0.5, AGENT_RADIUS, 5, 4);

    cs.add_circle(1.0, 1.0, 0.15, AGENT_RADIUS);
    cs.add_circle(3.0, 5.0, 0.12, AGENT_RADIUS);
    cs.add_circle(5.0, 5.0, 0.12, AGENT_RADIUS);
    cs.add_circle(9.0, 0.0, 0.12, AGENT_RADIUS);

    cs.add_lines(route, ROUTE_PADDING, AGENT_RADIUS);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        if (route.is_done()) {
            break;
        }
        // Sense

        // Plan
        ORRT orrt(agent.center, route.current_goal(), SAMPLING_PADDING);
        orrt.grow_tree(500, cs);
        agent.set_path(orrt.path_to_nearest_node_from_finish());

        // Act
        bool reached_finish = agent.update(0.01, cs);
        if (reached_finish) {
            route.increment_goal();
        }

        // Draw
        cs.draw(sense_viz);
        orrt.draw(plan_viz);
        agent.draw(act_viz);

        // Sleep
        loop_rate.sleep();
    }

    return 0;
}
