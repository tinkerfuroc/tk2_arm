#include "tk_arm/base_astar_planner.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tk_arm/ArmReachObjectAction.h>
#include <tk_arm/ArmPathAction.h>
#include <cstdio>

using namespace tinker::arm;

bool success;
tk_arm::ArmPathGoal goal;

namespace tinker{
namespace arm{
class ArmPointServer {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<tk_arm::ArmReachObjectAction> as_;
    actionlib::SimpleActionClient<tk_arm::ArmPathAction> ac_;
    tk_arm::ArmReachObjectResult result_;
    BaseAStarPlanner astar_planner;
    geometry_msgs::Point p0, p1;

public:
    ArmPointServer():
        nh_(), as_(nh_, "arm_point", boost::bind(&ArmPointServer::PointCallback, this, _1), false),
        ac_("arm_path", true) {
        p0.x = 0.38;
        p0.y = -0.02;
        p0.z = 0.15;
        ROS_INFO("Waiting for action server to start.");
        as_.start();
        ac_.waitForServer();
        ROS_INFO("Action server started.");
    }

    ~ArmPointServer() {
    }

    void PointCallback(const tk_arm::ArmReachObjectGoalConstPtr &new_goal) {
        p1 = new_goal->pos.point;
        goal.path = astar_planner.GetPath(p0, p1, success);
        ac_.sendGoal(goal);
        ROS_INFO("Waiting for result...");
        bool finished_before_timeout = ac_.waitForResult();
        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = ac_.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
            p0 = p1;
        }
        else {
            ROS_INFO("Action did not finish before the time out.");
            p0 = ac_.getResult()->moved;
        }
    }
};
}
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "arm_point_planner");
    ArmPointServer as;
    ros::spin();
    return 0;
}
