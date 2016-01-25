#include "tk_arm/arm_controller.h"
#include "tk_arm/OutPos.h"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <cmath>
#include <assert.h>
#include <ros/ros.h>


namespace tinker
{
namespace arm
{

const int ArmController::kNumJoint = 4;
const int ArmController::kNumSegment = 3;

//Length Factor is used for KDL library for better solve result
const int ArmController::kLengthFactor = 10;
const int ArmController::kAngleFactor = 0;
const int ArmController::kErrorRetry = 10;
const double ArmController::kMoveStep = 0.005;

ArmController::ArmController(const ArmInfo &arminfo)
    :arm_info_(arminfo), now_joint_angles_(kNumJoint - 1), 
    grasp_wait_time_(0), need_start_(false)
{
    ros::NodeHandle nh;
    ros::Rate rate(10);
    position_pub_ = nh.advertise<tk_arm::OutPos>("arm_pos", 1);
    assert(arminfo.min_angles.size() == kNumJoint);
    assert(arminfo.max_angles.size() == kNumJoint);
    assert(arminfo.init_angles.size() == kNumJoint);
    assert(arminfo.segments.size() == kNumSegment);
    for(int i = 0; i < kNumSegment; i++)
    {
        chain_.addSegment(arminfo.segments[i]);
    }
    for(int i = 0; i < kNumJoint - 1; i++)
    {
        now_joint_angles_(i) = arminfo.init_angles[i];
    }
    now_end_point_ = CalculateEndPostion();
    for(int i = 0; i < 10; i++)
    {
        PublishNowPose();
        rate.sleep();
    }
}

void ArmController::SetTarget(const geometry_msgs::Point & point)
{
    target_end_point_ = point;
    need_start_ = true;
}

void ArmController::PublishNowPose()
{
    tk_arm::OutPos msg;
    msg.pos1 = now_joint_angles_(0);
    msg.pos2 = now_joint_angles_(1);
    msg.pos3 = now_joint_angles_(2);
    msg.pos4 = M_PI / 2 - msg.pos2 - msg.pos3;
    msg.pos5 = 0;
    msg.pos6 = 0;
    ROS_INFO("OUTPOS:%lf %lf %lf %lf %lf %lf", 
            msg.pos1 / M_PI * 180.0, 
            msg.pos2 / M_PI * 180.0, 
            msg.pos3 / M_PI * 180.0, 
            msg.pos4 / M_PI * 180.0, 
            msg.pos5 / M_PI * 180.0, 
            msg.pos6 / M_PI * 180.0);
    position_pub_.publish(msg);
}

bool ArmController::NeedStart()
{
    return need_start_;
}

bool ArmController::GetNewTarget(geometry_msgs::Point &new_target)
{
    double x = target_end_point_.x - now_end_point_.x;
    double y = target_end_point_.y - now_end_point_.y;
    double z = target_end_point_.z - now_end_point_.z;
    double distance = sqrt(x*x + y*y + z*z);
    new_target.x = now_end_point_.x + x * kMoveStep / distance;
    new_target.y = now_end_point_.y + y * kMoveStep / distance;
    new_target.z = now_end_point_.z + z * kMoveStep / distance;
    return true;
}

void ArmController::SetJointAngle(const KDL::JntArray &joint_angle)
{
    need_start_ = false;
    PublishNowPose();
    now_joint_angles_ = joint_angle;
    now_end_point_ = CalculateEndPostion();
}

bool ArmController::CalculateJointAngle(const geometry_msgs::Point &target, KDL::JntArray &joint_angle)
{
    double x = target_end_point_.x - now_end_point_.x;
    Eigen::Matrix<double, 2*(kNumJoint-1), 1> constraint;
    for(int i=0; i < kNumJoint - 1; i++)
    {
        constraint(i) = 1;
        constraint(i + kNumJoint - 1) = 0;
    }
    KDL::ChainIkSolverPos_LMA solver(chain_, constraint);
    KDL::JntArray new_joint_angles(kNumJoint - 1);
    KDL::JntArray seed_joint_angles = now_joint_angles_;
    int retval = -1;
    KDL::Frame pos_goal(KDL::Vector(target.x * double(kLengthFactor), 
                target.y * double(kLengthFactor), 
                target.z * double(kLengthFactor)));
    for(int i=0; i<kErrorRetry; i++)
    {
        retval = solver.CartToJnt(seed_joint_angles, pos_goal, new_joint_angles);
        if(retval != 0)
        {
            WarnKDLSolve(retval);
            continue;
        }
        if(CheckAngleLegal(new_joint_angles))
        {
            joint_angle = new_joint_angles;
            return true;
        }
        seed_joint_angles.data.setRandom();
        seed_joint_angles.data *= M_PI;
    }
    ROS_ERROR("failed to calculate joint angle");
    ROS_ERROR("now position %lf, %lf, %lf", now_end_point_.x, now_end_point_.y, now_end_point_.z);
    ROS_ERROR("target position %lf, %lf, %lf", target.x, target.y, target.z);
    return false;
}

bool ArmController::HasArrivedTarget()
{
    double errorx = now_end_point_.x - target_end_point_.x;
    double errory = now_end_point_.y - target_end_point_.y;
    double errorz = now_end_point_.z - target_end_point_.z;
    return sqrt(errorx * errorx + errory * errory + errorz * errorz) < 0.02;
}

void ArmController::OnArrive()
{
    ROS_INFO("Arrived!");
}

geometry_msgs::Point ArmController::CalculateEndPostion()
{
    KDL::ChainFkSolverPos_recursive fksolver(chain_);
    KDL::Frame pos;
    fksolver.JntToCart(now_joint_angles_, pos);
    geometry_msgs::Point end_point;
    end_point.x = pos.p.x() / double(kLengthFactor);
    end_point.y = pos.p.y() / double(kLengthFactor);
    end_point.z = pos.p.z() / double(kLengthFactor);
    ROS_INFO("now end position: %lf, %lf, %lf", end_point.x, end_point.y, end_point.z);
    return end_point;
}

void ArmController::WarnKDLSolve(int retval)
{
    ROS_WARN("Failed to solve joint angles");
    switch(retval)
    {
    case -1:
        ROS_WARN("Error Code -1. Gradiant too small.");
        break;
    case -2:
        ROS_WARN("Error Code -2. Joint pos increment too small.");
        break;
    case -3:
        ROS_WARN("Error Code -3. Iteration number exceeded.");
        break;
    default:
        break;
    }
}

bool ArmController::CheckAngleLegal(const KDL::JntArray &joint_angle)
{
    bool is_legal = true;
    for(int i=0; i<kNumJoint-1; i++)
    {
        if(joint_angle(i) > arm_info_.max_angles[i] || 
                joint_angle(i) < arm_info_.min_angles[i])
        {
            ROS_WARN("Angle out of bound for joint %d", i);
            is_legal = false;
        }
    }
    double angle3 = M_PI / 2 - joint_angle(2) - joint_angle(1);
    if(angle3 > arm_info_.max_angles[3] || 
            angle3 < arm_info_.min_angles[3])
    {
        ROS_WARN("Angle out of bound for joint 3");
        is_legal = false;
    }
    return is_legal;
}

}
}
