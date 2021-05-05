#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "cs.hpp"
#include "obstacles/circle.hpp"
#include "orrt.hpp"
#include "route.hpp"
#include "vec2.hpp"

using namespace std;

const float AGENT_RADIUS = 0.25;
const Vec2 start(0, 0);
const Vec2 finish(5, 5);
ConfigurationSpace cs;

int main(int argc, char** argv) {
    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist(0, 1);

    ros::init(argc, argv, "plan_node");

    ros::NodeHandle n;
    ros::Publisher rrt_viz = n.advertise<visualization_msgs::MarkerArray>("/plan/debug", 30);
    ros::Publisher cs_viz = n.advertise<visualization_msgs::MarkerArray>("/cs/debug", 1);

    cs.add_circle(1.0, 1.0, 0.25, AGENT_RADIUS);
    cs.add_circle(2.0, 2.0, 0.20, AGENT_RADIUS);

    Route route({Vec2(0.0, 0.0), Vec2(4.0, 4.0)});
    cs.add_lines(route.points, 1.5, AGENT_RADIUS);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        // Grow tree
        ORRT orrt(start, finish);
        std::vector<Vec2> points;
        for (int i = 0; i < 500; ++i) {
            points.push_back(Vec2(dist(e2) * 5.0, dist(e2) * 5.0));
        }
        orrt.grow_tree(points, cs);
        // Draw RRT*
        orrt.draw(rrt_viz);
        cs.draw(cs_viz);
        // Sleep
        loop_rate.sleep();
    }

    return 0;
}
