#include <tk_arm/hand_controller.h>
#include <std_msgs/Float64.h>
#include <assert.h>
#include <ros/ros.h>

using namespace tinker::arm;

const int HandController::kGraspWaitTime = 1;

namespace tinker {
namespace arm {
HandController::HandController(std::string server_name_)
    : nh_(),
      as_(nh_, "arm_hand",
          boost::bind(&HandController::HandCallback, this, _1), false),
      rate_(10), in_grasp_(false) {
    hand_pub_ = nh_.advertise<std_msgs::Float64>("/claw_controller/command", 0);
    as_.start();
    ROS_INFO("Hand Server started.");
    ROS_INFO("Hand Controller Initiated.");
}

void HandController::HandCallback(const tk_arm::ArmHandGoalConstPtr &new_goal) {
    if (new_goal->state)
        GraspObject();
    else
        ReleaseObject();

    result_.hand_closed = in_grasp_;

    if (in_grasp_ == new_goal->state) {
        as_.setSucceeded(result_);
        ROS_INFO("Action Succeded.");
    } else {
        as_.setAborted(result_);
        ROS_INFO("Action Aborted.");
    }
}

void HandController::MoveHand() {
    ROS_INFO("\033[0;34mPublishing Hand Control\033[0;0m");
    std_msgs::Float64 msg;
    msg.data = in_grasp_ ? -100.0 : 20.0;
    ROS_INFO("Hand torque %lf", msg.data);
    hand_pub_.publish(msg);
}

bool HandController::GraspObject() {
    ros::Duration(kGraspWaitTime).sleep();
    in_grasp_ = true;
    MoveHand();
    ROS_INFO("\033[1;35mObject grasped!\033[0;0m");
    ros::Duration(kGraspWaitTime).sleep();
    return true;
}

bool HandController::ReleaseObject() {
    ros::Duration(kGraspWaitTime).sleep();
    in_grasp_ = false;
    MoveHand();
    ROS_INFO("\033[1;35mObject released!\033[0;0m");
    ros::Duration(kGraspWaitTime).sleep();
    return true;
}
}
}
