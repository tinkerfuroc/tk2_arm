#include "tk_arm/base_astar_planner.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tk_arm/ArmReachObjectAction.h>
#include <tk_arm/ArmPathAction.h>
#include <cstdio>
#include <actionlib_msgs/GoalStatus.h>

using namespace tinker::arm;
using actionlib_msgs::GoalStatus;

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
        p0.x = 0.41;
        p0.y = 0.;
        p0.z = 0.;
        as_.start();
        ROS_INFO("Action server 'arm_point' started.");
        ac_.waitForServer();
        ROS_INFO("Action client 'arm_path' started.");
    }

    ~ArmPointServer() {
    }

    void PointCallback(const tk_arm::ArmReachObjectGoalConstPtr &new_goal) {
        bool success;
        bool need_y_move;
        tk_arm::ArmPathGoal goal;
        p1 = new_goal->pos.point;
        if (p1.y != 0)
            need_y_move = true;
        p1.y = 0;
        goal.path = astar_planner.GetPath(p0, p1, success);
        success = success && (!need_y_move);
        if (goal.path.poses.size() > 0)
            p0 = goal.path.poses.back().pose.position;
        ac_.sendGoal(goal);
        ros::Rate r(20);
        while(true) {
            if (ac_.getState().isDone())
                break;
            if (as_.isPreemptRequested()) {
                ac_.cancelAllGoals();
                as_.setPreempted();
                return;
            }
            r.sleep();
        }
        actionlib::SimpleClientGoalState state = ac_.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        ROS_INFO("Now at %f %f %f", p0.x, p0.y, p0.z);
        result_.moved = ac_.getResult()->moved;
        result_.is_reached = success;
        if (result_.is_reached) 
            as_.setSucceeded(result_);
        else
            as_.setAborted(result_);
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
