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

struct ObstaclesPublisher {
    std::vector<Circle> circles;

    ObstaclesPublisher() {
        circles.push_back(Circle(Vec2(0.0, 3.0), 0.1));
        circles.push_back(Circle(Vec2(3.0, 5.0), 0.1));
        circles.push_back(Circle(Vec2(8.0, 1.0), 0.1));
        circles.push_back(Circle(Vec2(11.0, 2.0), 0.1));
        circles.push_back(Circle(Vec2(11.0, 5.0), 0.1));
        circles.push_back(Circle(Vec2(11.0, 4.0), 0.1));
        circles.push_back(Circle(Vec2(11.0, 3.0), 0.1));
        circles.push_back(Circle(Vec2(8.0, 2.0), 0.1));
        circles.push_back(Circle(Vec2(8.0, 5.0), 0.1));
        circles.push_back(Circle(Vec2(8.0, 4.0), 0.1));
        circles.push_back(Circle(Vec2(8.0, 3.0), 0.1));
    }

    std::vector<Circle> get_obstacles(const Vec2& position, const float range) const {
        std::vector<Circle> ans;
        for (const Circle& circle : circles) {
            if ((circle.center - position).norm() < range) {
                ans.push_back(circle);
            }
        }
        return ans;
    }
};

const float AGENT_RADIUS = 0.25;
const float ROUTE_PADDING = AGENT_RADIUS * 6;
const float SAMPLING_PADDING = ROUTE_PADDING * 1.2;
const float SENSING_RANGE = 2.0;
ConfigurationSpace cs;

int main(int argc, char** argv) {
    ros::init(argc, argv, "plan_node");

    ros::NodeHandle n;
    ros::Publisher plan_viz = n.advertise<visualization_msgs::MarkerArray>("/plan/debug", 30);
    ros::Publisher sense_viz = n.advertise<visualization_msgs::MarkerArray>("/sense/debug", 1);
    ros::Publisher act_viz = n.advertise<visualization_msgs::MarkerArray>("/act/debug", 1);

    // Input
    Route route({Vec2(0.0, 0.0), Vec2(0.0, 5.0), Vec2(5.0, 5.0), Vec2(7.0, 5.0), Vec2(9.0, 0.0), Vec2(12.0, 5.0), Vec2(12.0, 8.0), Vec2(18.0, 8.0)});
    DiffDrive agent(route.start(), 0.5, AGENT_RADIUS, 3, 3);

    ObstaclesPublisher obs_pub;
    cs.add_lines(route, ROUTE_PADDING, AGENT_RADIUS);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        // Done?
        if (route.is_done()) {
            break;
        }

        // Sense
        cs.clear_circles();
        for (const Circle& sensed_circle : obs_pub.get_obstacles(agent.center, SENSING_RANGE)) {
            cs.add_circle(sensed_circle.center.x, sensed_circle.center.y, sensed_circle.radius, AGENT_RADIUS);
        }

        // Plan
        ORRT orrt(agent.center, route.current_goal(), SAMPLING_PADDING);
        orrt.grow_tree(500, cs);
        agent.set_path(orrt.path_to_nearest_node_from_finish());

        // Act
        for (int i = 0; i < 10; i++) {
            bool reached_finish = agent.update(0.01, cs);
            if (reached_finish) {
                route.increment_goal();
                break;
            }
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
