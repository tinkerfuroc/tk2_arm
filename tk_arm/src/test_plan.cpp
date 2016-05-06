#include "tk_arm/base_astar_planner.h"

using namespace tinker::arm;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_plan");
    BaseAStarPlanner astar_planner;
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    geometry_msgs::Point p3;
    geometry_msgs::Point p4;
    p1.x = 0.5;
    p1.y = -0.2;
    p1.z = 0;
    p2.x = 0.6;
    p2.y = 0.1;
    p2.z = 0.3;
    p3.x = 0.7;
    p3.y = 0.2;
    p3.z = -0.5;
    p4.x = 1.0;
    p4.y = 0;
    p4.z = 0.8;
    bool success;
    astar_planner.GetPath(p1, p2, success);
    astar_planner.GetPath(p1, p3, success);
    astar_planner.GetPath(p2, p3, success);
    astar_planner.GetPath(p1, p4, success);
    return 0;
}

