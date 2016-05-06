#include "tk_arm/simple_arm_controller.h"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <std_msgs/Float64.h>
#include <cmath>
#include <assert.h>
#include <ros/ros.h>

using namespace tinker::arm;
using KDL::Joint;
using KDL::Segment;
using KDL::Frame;
using KDL::Vector;

const int SimpleArmController::kGraspWaitTime = 1;
const double SimpleArmController::kMoveStep = 0.01;
const double SimpleArmController::kDegreeInterpolation = (1.0 / 180.0 * M_PI);
const double SimpleArmController::kShoulderMoveStep = 0.05;

namespace tinker {
namespace arm {
SimpleArmController::SimpleArmController(std::string server_name_)
    : nh_(),
      as_(nh_, "arm_reach_position",
          boost::bind(&SimpleArmController::PositionCallback, this, _1), false),
      as_init_(nh_, "arm_reset",
               boost::bind(&SimpleArmController::InitCallback, this, _1),
               false),
      rate_(10),
      current_joint_angles_(ArmIK::kNumJoint - 1),
      target_joint_angles_(ArmIK::kNumJoint - 1),
      in_grasp_(false),
      current_height_(0.0),
      in_init_(false) {
    base_pub_ = nh_.advertise<std_msgs::Float64>(
        "/base_joint_position_controller/command", 0);
    shoulder_rotation_pub_ = nh_.advertise<std_msgs::Float64>(
        "/shoulder_rotation_joint_position_controller/command", 0);
    shoulder_flexion_pub_ = nh_.advertise<std_msgs::Float64>(
        "/shoulder_flexion_joint_position_controller/command", 0);
    elbow_pub_ = nh_.advertise<std_msgs::Float64>(
        "/elbow_joint_position_controller/command", 0);
    wrist_deviation_pub_ = nh_.advertise<std_msgs::Float64>(
        "/wrist_deviation_controller/command", 0);
    wrist_extension_pub_ = nh_.advertise<std_msgs::Float64>(
        "/wrist_extension_controller/command", 0);
    hand_pub_ = nh_.advertise<std_msgs::Float64>("/claw_controller/command", 0);

    for (int i = 0; i < ArmIK::kNumJoint - 1; ++i) {
        current_joint_angles_(i) = ArmIK::SEG_INIT[i];
        target_joint_angles_(i) = ArmIK::SEG_INIT[i];
        initial_joint_angles_(i) = ArmIK::SEG_INIT[i];
    }

    current_end_point_ = arm_ik_.AngleToPosition(current_joint_angles_);
    ROS_INFO("Parameters set.");

    // start actionlib servers
    as_.start();
    as_init_.start();
    ROS_INFO("Server started.");
    TurnShoulder();
}

void SimpleArmController::PositionCallback(
    const tk_arm::ArmReachObjectGoalConstPtr &new_goal) {
    geometry_msgs::Point start_point = current_end_point_;
    if (new_goal->pos.header.frame_id == "arm_origin_link") {
        ROS_WARN("Forcing frame id to be arm_origin_link");
    }

    need_grasp_ = new_goal->state & 0x03;

    if (need_grasp_ & 0x02) {
        if (need_grasp_ & 0x01)
            GraspObject();
        else
            ReleaseObject();
        result_.moved = current_end_point_;
        result_.is_reached = true;
        as_.setSucceeded(result_);
        return;
    }

    geometry_msgs::Point origin_current_end_point_ = current_end_point_;
    KDL::JntArray origin_current_joint_angles_ = current_joint_angles_;

    double x = new_goal->pos.point.x;
    double y = new_goal->pos.point.y;
    double z = new_goal->pos.point.z;
    object_end_point_.x = x;
    object_end_point_.y = y;
    object_end_point_.z = z;

    bool test_state = new_goal->state & 0x04;
    need_grasp_ = new_goal->state & 0x03;
    bool success = true;

    if (test_state) ROS_INFO(">>>>> TEST STATE <<<<<");
    ROS_INFO("New Goal! [%4.2lf %4.2lf %4.2lf]", object_end_point_.x,
             object_end_point_.y, object_end_point_.z);
    if (!HasArrivedObject()) {
        result_.is_reached = GoToPosition(!test_state);
        if (!result_.is_reached) {
            ROS_INFO("Go to position failed.");
            success = false;
        }
    }
    if ((!test_state) && success) {
        if (need_grasp_ & 0x01)
            GraspObject();
        else
            ReleaseObject();
    }
    result_.moved = current_end_point_;
    result_.is_reached = success;
    if (success) {
        as_.setSucceeded(result_);
        ROS_INFO("Action Succeded.");
    } else {
        as_.setAborted(result_);
        ROS_INFO("Action Aborted.");
    }

    if (test_state) {
        current_end_point_ = origin_current_end_point_;
        current_joint_angles_ = origin_current_joint_angles_;
        ROS_INFO(">>>>> TEST STATE ENDS <<<<<");
    }
}

void SimpleArmController::InitCallback(
    const tk_arm::ArmInitGoalConstPtr &new_goal) {
    ROS_INFO("Go Init! [%d]", new_goal->state);
    result_init_.is_reached = GoInit();
    ROS_INFO("%s",
             result_init_.is_reached ? "Go init success." : "Go init failed.");
    as_init_.setSucceeded(result_init_);
}

bool SimpleArmController::GoInit() {
    target_height_ = ArmIK::kBaseHeightMin;
    MoveBase(1);
    // set goal to init angles
    for (int i = 0; i < ArmIK::kNumJoint - 1; i++) {
        target_joint_angles_(i) = ArmIK::SEG_INIT[i];
    }
    target_end_point_ = arm_ik_.AngleToPosition(target_joint_angles_);

    // interpolation in motor angle
    double maxDegree = 0.0;
    int interpolationNum = 0.0;
    for (int i = 0; i < 3; ++i) {
        if (fabs(target_joint_angles_(i) - current_joint_angles_(i)) >
            maxDegree)
            maxDegree =
                fabs(target_joint_angles_(i) - current_joint_angles_(i));
    }
    interpolationNum = maxDegree / kDegreeInterpolation + 1;
    std::vector<double> msg;
    msg.resize(ArmIK::kNumJoint);
    KDL::JntArray initial_joint_angles_(current_joint_angles_);
    for (int i = 0; i < interpolationNum; ++i) {
        current_joint_angles_(0) =
            (target_joint_angles_(0) * i +
             initial_joint_angles_(0) * (interpolationNum - i)) /
            interpolationNum;
        current_joint_angles_(1) =
            (target_joint_angles_(1) * i +
             initial_joint_angles_(1) * (interpolationNum - i)) /
            interpolationNum;
        current_joint_angles_(2) =
            (target_joint_angles_(2) * i +
             initial_joint_angles_(2) * (interpolationNum - i)) /
            interpolationNum;

        MoveArm();
        ros::spinOnce();
        rate_.sleep();
    }
    current_joint_angles_(0) = target_joint_angles_(0);
    current_joint_angles_(1) = target_joint_angles_(1);
    current_joint_angles_(2) = target_joint_angles_(2);
    MoveArm();

    in_init_ = true;
    return true;
}

bool SimpleArmController::GoToPosition(bool move) {
    // interpolate on current-to-object direction
    ROS_INFO("Go to position [%4.2lf %4.2lf %4.2lf]", object_end_point_.x,
             object_end_point_.y, object_end_point_.z);
    if (in_init_) TurnShoulder();
    bool is_ok = true;
    std::vector<double> msg;
    msg.resize(ArmIK::kNumJoint);
    target_height_ =
        std::max(std::min(object_end_point_.z - ArmIK::kBaseHeightDiff,
                          ArmIK::kBaseHeightMax),
                 ArmIK::kBaseHeightMin);
    MoveBase(move);
    ROS_INFO(
        "Go to position [%4.2lf %4.2lf %4.2lf] from [%4.2lf %4.2lf %4.2lf]...",
        object_end_point_.x, object_end_point_.y, object_end_point_.z,
        current_end_point_.x, current_end_point_.y, current_end_point_.z);

    while (!HasArrivedObject()) {
        double x = object_end_point_.x - current_end_point_.x;
        double y = object_end_point_.y - current_end_point_.y;
        double z = object_end_point_.z - current_end_point_.z;
        double distance = sqrt(x * x + y * y + z * z);
        target_end_point_.x = current_end_point_.x + x * kMoveStep / distance;
        target_end_point_.y = current_end_point_.y + y * kMoveStep / distance;
        target_end_point_.z = current_end_point_.z + z * kMoveStep / distance;
        if (!arm_ik_.PositionToAngle(target_end_point_, target_joint_angles_, 10)) {
            ROS_WARN("Position to angle failed.");
            return false;
        }
        current_joint_angles_ = target_joint_angles_;

        if (move) {
            MoveArm();
            ros::spinOnce();
            rate_.sleep();
        } else {
            current_end_point_ = arm_ik_.AngleToPosition(current_joint_angles_);
        }
    }
    return is_ok;
}

void SimpleArmController::TurnShoulder() {
    while (fabs(current_joint_angles_(0)) > 0.05) {
        current_joint_angles_(0) += kShoulderMoveStep;
        MoveArm();
        ros::spinOnce();
        rate_.sleep();
    }
    in_init_ = false;
}

bool SimpleArmController::MoveBase(bool move) {
    std_msgs::Float64 msg;
    bool direction = target_height_ > current_height_;
    ROS_INFO("Move base to %4.2lf. Current base height: %4.2lf. %s.",
             target_height_, current_height_,
             direction ? "Moving upwards."
                       : (target_height_ < current_height_ ? "Moving downwards."
                                                           : "Not moving."));

    if (move) {
        msg.data = target_height_;
        base_pub_.publish(msg);
        ros::Duration(50 * fabs(target_height_ - current_height_) + 2).sleep();
        current_height_ = target_height_;
        ROS_INFO("\033[0;35mBase moved to %4.2lf.\033[0;0m", current_height_);
    }

    object_end_point_.z = object_end_point_.z - target_height_;
    ROS_INFO("Move base succedded.");
    return true;
}

void SimpleArmController::MoveArm() {
    ROS_INFO("\033[0;34mPublishing angle: %4.2lf %4.2lf %4.2lf %4.2lf\033[0;0m",
             current_joint_angles_(0) * 180 / M_PI,
             (M_PI / 2 - current_joint_angles_(1)) * 180 / M_PI,
             (current_joint_angles_(2)) * 180 / M_PI,
             (M_PI / 2 + current_joint_angles_(1) + current_joint_angles_(2)) *
                 180 / M_PI);
    current_end_point_ = arm_ik_.AngleToPosition(current_joint_angles_);
    std_msgs::Float64 msg;
    msg.data = current_joint_angles_(0);
    shoulder_rotation_pub_.publish(msg);
    msg.data = M_PI / 2 - current_joint_angles_(1);
    shoulder_flexion_pub_.publish(msg);
    msg.data = current_joint_angles_(2);
    elbow_pub_.publish(msg);
    msg.data = M_PI;
    wrist_deviation_pub_.publish(msg);
    msg.data =
        M_PI / 2 + current_joint_angles_(1) + current_joint_angles_(2) + 0.12;
    wrist_extension_pub_.publish(msg);
    msg.data = in_grasp_ ? -100 : 20;
    ROS_INFO("Hand torque %lf", msg.data);
    hand_pub_.publish(msg);
}

bool SimpleArmController::HasArrivedObject() {
    error_.x = current_end_point_.x - object_end_point_.x;
    error_.y = current_end_point_.y - object_end_point_.y;
    error_.z = current_end_point_.z - object_end_point_.z;
    return sqrt(error_.x * error_.x + error_.y * error_.y +
                error_.z * error_.z) < 0.01;
}

bool SimpleArmController::HasArrivedTarget() {
    error_.x = current_end_point_.x - target_end_point_.x;
    error_.y = current_end_point_.y - target_end_point_.y;
    error_.z = current_end_point_.z - target_end_point_.z;
    return sqrt(error_.x * error_.x + error_.y * error_.y +
                error_.z * error_.z) < 0.01;
}

bool SimpleArmController::GraspObject() {
    ros::Duration(kGraspWaitTime).sleep();
    in_grasp_ = true;
    MoveArm();
    ROS_INFO("\033[1;35mObject grasped!\033[0;0m");
    ros::Duration(kGraspWaitTime).sleep();
    return true;
}

bool SimpleArmController::ReleaseObject() {
    ros::Duration(kGraspWaitTime).sleep();
    in_grasp_ = false;
    MoveArm();
    ROS_INFO("\033[1;35mObject released!\033[0;0m");
    ros::Duration(kGraspWaitTime).sleep();
    return true;
}
}
}
