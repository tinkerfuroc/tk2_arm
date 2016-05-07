#include "tk_arm/base_astar_planner.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tk_arm/ArmPathAction.h>
#include <cstdio>

using namespace tinker::arm;

bool success;
tk_arm::ArmPathGoal goal;

void sendMsg(actionlib::SimpleActionClient<tk_arm::ArmPathAction> &ac);

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_plan");
    actionlib::SimpleActionClient<tk_arm::ArmPathAction> ac("arm_path", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started.");

    BaseAStarPlanner astar_planner;
    geometry_msgs::Point p[4];
    p[0].x = 0.5;
    p[0].y = 0;
    p[0].z = 0;

    p[1].x = 0.8;
    p[1].y = 0.1;
    p[1].z = 0.3;

    p[2].x = 0.1;
    p[2].y = 0.2;
    p[2].z = -0.5;

    p[3].x = 0.64;
    p[3].y = 0;
    p[3].z = 0.8;

    goal.path = astar_planner.GetPath(p[0], p[1], success);
    sendMsg(ac);
    goal.path = astar_planner.GetPath(p[0], p[2], success);
    sendMsg(ac);
    goal.path = astar_planner.GetPath(p[1], p[2], success);
    sendMsg(ac);
    goal.path = astar_planner.GetPath(p[3], p[2], success);
    sendMsg(ac);

    return 0;
}

void sendMsg(actionlib::SimpleActionClient<tk_arm::ArmPathAction> &ac){    
    getchar();
    ac.sendGoal(goal);

    ROS_INFO("Waiting for result...");
    bool finished_before_timeout = ac.waitForResult();
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else
        ROS_INFO("Action did not finish before the time out.");
}