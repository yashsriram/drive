#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "cs.hpp"
#include "obstacle.hpp"
#include "orrt.hpp"
#include "vec2.hpp"

using namespace std;

const float AGENT_RADIUS = 0.25;
const Vec2 start(0, 0);
const Vec2 finish(5, 5);
ConfigurationSpace cs(AGENT_RADIUS);

void update_obstacles(const visualization_msgs::MarkerArray& msg) {
    cs.obstacles.clear();
    for (const auto& m : msg.markers) {
        cs.obstacles.push_back(Obstacle(Vec2(m.pose.position.x, m.pose.position.y), max(m.scale.x, m.scale.y) / 2.0));
    }
    std::cout << "got " << cs.obstacles.size() << " obstacles" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plan_node");

    ros::NodeHandle n;
    /* ros::Subscriber obstacles_sub = n.subscribe("/obstacles_publisher/obstacles", 1, update_obstacles); */
    ros::Publisher rrt_viz = n.advertise<visualization_msgs::MarkerArray>("/plan/debug", 30);

    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist(0, 1);

    ros::Rate loop_rate(30);
    /* for (int i = 0; i < 50; ++i) { */
    /*     // Get obstacles */
    /*     ros::spinOnce(); */
    /*     loop_rate.sleep(); */
    /* } */
    while (ros::ok()) {
        // Grow tree
        ORRT orrt(rrt_viz, start, finish);
        std::vector<Vec2> points;
        for (int i = 0; i < 500; ++i) {
            points.push_back(Vec2(dist(e2) * 5.0, dist(e2) * 5.0));
        }
        orrt.grow_tree(points, cs);
        // Draw RRT*
        orrt.draw();
        // Sleep
        loop_rate.sleep();
    }

    return 0;
}
