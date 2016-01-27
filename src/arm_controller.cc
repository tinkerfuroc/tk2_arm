#include "tk_arm/arm_controller.h"
#include "tk_arm/OutPos.h"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <std_msgs/String.h>
#include <cmath>
#include <assert.h>
#include <ros/ros.h>
#include <iostream>

namespace tinker
{
namespace arm
{

const int ArmController::kNumJoint = 4;
const int ArmController::kNumSegment = 3;
const int ArmController::kErrorRetry = 10;

//Length Factor is used for KDL library for better solve result
const double ArmController::kLengthFactor = 10.0;
const double ArmController::kAngleFactor = 0.0;
const double ArmController::kMoveStep = 0.005;
const double ArmController::kCorrectionFactor = 0.0002;
const int ArmController::kCorrectionCount = 5;

const double ArmController::kImageWidth = 640.0;
const double ArmController::kImageHeight = 480.0;
const double ArmController::kForwardVelocity = 0.05;
const double ArmController::kBlindDistance = 0.05;

ArmController::ArmController(const ArmInfo &arminfo)
	: arm_info_(arminfo), now_joint_angles_(kNumJoint - 1),
	  grasp_wait_time_(1), need_start_(false), in_grasp_(false), aligned_with_object_(false),
	  error_last_time_(false), correction_updated_count_(0),
	  in_reaching_(false), in_retreiving_(false), in_init_(false), in_duck_(true),
      need_grasp_(false)
{
	ros::NodeHandle nh;
	ros::Rate rate(10);
	position_pub_ = nh.advertise<tk_arm::OutPos>("arm_pos", 1);
	mission_pub_ = nh.advertise<std_msgs::String>("arm_arrive", 1);
	position_sub_ = nh.subscribe("tk2_vision/arm_target_finder/caught_obj", 1, &ArmController::UpdateCVCorrection, this);
	assert(arminfo.min_angles.size() == kNumJoint);
	assert(arminfo.max_angles.size() == kNumJoint);
	assert(arminfo.init_angles.size() == kNumJoint);
	assert(arminfo.duck_angles.size() == kNumJoint);
	assert(arminfo.segments.size() == kNumSegment);
	for(int i = 0; i < kNumSegment; i++)
	{
		chain_.addSegment(arminfo.segments[i]);
	}
	for(int i = 0; i < kNumJoint - 1; i++)
	{
		now_joint_angles_(i) = arminfo.duck_angles[i];
	}
	now_end_point_ = CalculateEndPostion(now_joint_angles_);
	SetTarget(now_end_point_);
	for(int i = 0; i < 10; i++)
	{
		PublishNowPose();
		rate.sleep();
	}
    need_start_ = false;
}

void ArmController::GoalCallback(geometry_msgs::Point::ConstPtr new_goal)
{
    ROS_INFO("New Goal!");
    if(new_goal->x < 0.1 && new_goal->y < 0.1 && new_goal->z < 0.1)
    {
        return;
    }
    object_end_point_ = *new_goal;
    if(in_duck_)
    {
        GoInit();
        in_duck_ = false;
        in_init_ = true;
        need_grasp_ = true;
        return;
    }
    if(in_init_)
    {
        need_grasp_ = true;
    }
}

void ArmController::SetTarget(const geometry_msgs::Point &point)
{
	target_end_point_ = point;
	need_start_ = true;
}

void ArmController::SetObject(const geometry_msgs::Point &point)
{
	object_end_point_ = point;
	need_start_ = true;
}

void ArmController::GoInit()
{
	KDL::JntArray angles(kNumJoint - 1);
	for(int i = 0; i < kNumJoint - 1; i++)
	{
		angles(i) = arm_info_.init_angles[i];
	}
	geometry_msgs::Point point = CalculateEndPostion(angles);
	SetTarget(point);
}

void ArmController::GoDuck()
{
	KDL::JntArray angles(kNumJoint - 1);
	for(int i = 0; i < kNumJoint - 1; i++)
	{
		angles(i) = arm_info_.duck_angles[i];
	}
	geometry_msgs::Point point = CalculateEndPostion(angles);
	SetTarget(point);
}

void ArmController::PublishNowPose()
{
	tk_arm::OutPos msg;
	msg.pos1 = now_joint_angles_(0);
	msg.pos2 = now_joint_angles_(1);
	msg.pos3 = now_joint_angles_(2);
	msg.pos4 = M_PI / 2 - msg.pos2 - msg.pos3;
	msg.pos5 = 0;
	msg.pos6 = in_grasp_;
	position_pub_.publish(msg);
}

void ArmController::PublishMissionDone()
{
	std_msgs::String msg;
	msg.data = "Done.";
	ROS_INFO("%s", msg.data.c_str());
	mission_pub_.publish(msg);	
}

bool ArmController::TimeCallback()
{
	if (error_last_time_)
	{
		ROS_WARN("Cannot reach");
        GoInit();
        need_grasp_ = false;
        in_reaching_ = false;
        in_retreiving_ = false;
		in_init_ = true;
        in_duck_ = false;
        error_last_time_ = false;
	}
	if (HasArrivedTarget())
	{
        if (in_init_)
		{
            PublishMissionDone();
			if(need_grasp_ = true)
			{
				in_init_ = false;
				in_reaching_ = true;
			}
		}
		else if(in_reaching_)
		{
			geometry_msgs::Point point = target_end_point_;
			if (!aligned_with_object_)
			{
                double factor = now_end_point_.x / object_end_point_.x;
				point.y = object_end_point_.y * factor;
				point.z = object_end_point_.z * factor;
				SetTarget(point);
				aligned_with_object_ = true;
			}
			else if (point.x < object_end_point_.x)
			{
				if (point.x < object_end_point_.x - kBlindDistance)
				{
					point.x += kForwardVelocity;
					SetTarget(point);
				}
				else
				{
					printf("correction count: %d\n", correction_updated_count_);
					if (correction_updated_count_ >= kCorrectionCount)
					{
						point.x = object_end_point_.x;
						SetTarget(point);
					}
					else
					{
						GoInit();
                        need_grasp_ = false;
						in_reaching_ = false;
						in_init_ = true;
						aligned_with_object_ = false;                            	
					}
				}
			}
			else
			{
				GraspObject();
				GoInit();
                need_grasp_ = false;
				in_reaching_ = false;
				in_retreiving_ = true;
				aligned_with_object_ = false;
			}
		}
		else if(in_retreiving_)
		{
			ReleaseObject();
            GoInit();
            need_grasp_ = false;
			in_retreiving_ = false;
			in_init_ = true;
		}
	}
	BaseArmController::TimeCallback();
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
	double distance = sqrt(x * x + y * y + z * z);
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
	now_end_point_ = CalculateEndPostion(now_joint_angles_);
}

bool ArmController::CalculateJointAngle(const geometry_msgs::Point &target, KDL::JntArray &joint_angle)
{
	double x = target_end_point_.x - now_end_point_.x;
	Eigen::Matrix < double, 2 * (kNumJoint - 1), 1 > constraint;
	for(int i = 0; i < kNumJoint - 1; i++)
	{
		constraint(i) = 1;
		constraint(i + kNumJoint - 1) = kAngleFactor;
	}
	KDL::ChainIkSolverPos_LMA solver(chain_, constraint);
	KDL::JntArray new_joint_angles(kNumJoint - 1);
	KDL::JntArray seed_joint_angles = now_joint_angles_;
	int retval = -1;
	KDL::Frame pos_goal(KDL::Vector(target.x * kLengthFactor,
									target.y * kLengthFactor,
									target.z * kLengthFactor));
	for(int i = 0; i < kErrorRetry; i++)
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
			error_last_time_ = false;
			return true;
		}
		seed_joint_angles.data.setRandom();
		seed_joint_angles.data *= M_PI;
	}
	ROS_ERROR("failed to calculate joint angle");
	ROS_ERROR("target position %6.3lf, %6.3lf, %6.3lf", target.x, target.y, target.z);
	ROS_INFO("now position %6.3lf, %6.3lf, %6.3lf", now_end_point_.x, now_end_point_.y, now_end_point_.z);
	error_last_time_ = true;
	return false;
}

bool ArmController::HasArrivedTarget()
{
	error_.x = now_end_point_.x - target_end_point_.x;
	error_.y = now_end_point_.y - target_end_point_.y;
	error_.z = now_end_point_.z - target_end_point_.z;
	return sqrt(error_.x * error_.x + error_.y  * error_.y  + error_.z  * error_.z ) < 0.01;
}

void ArmController::OnArriveTarget()
{
	ROS_INFO("Target Arrived!");
}

bool ArmController::GraspObject()
{
	if (!in_grasp_)
	{
		ros::Duration(grasp_wait_time_).sleep();
		in_grasp_ = true;
		PublishNowPose();
		ROS_INFO("Target grasped!");
		ros::Duration(grasp_wait_time_).sleep();
		return true;
	}
	return false;
}

bool ArmController::ReleaseObject()
{
	if (in_grasp_)
	{
		ros::Duration(grasp_wait_time_).sleep();
		in_grasp_ = false;
		PublishNowPose();
		ROS_INFO("Target released!");
		ros::Duration(grasp_wait_time_).sleep();
		return true;
	}
	return false;
}

void ArmController::UpdateCVCorrection(const tk_arm::TargetFound &msg)
{
	if (msg.object_recognized)
	{
		correction_updated_count_++;
		double center_left = msg.object_rect_left + msg.object_rect_width * 0.5;
		double center_top = msg.object_rect_top + msg.object_rect_height * 0.5;
		corr_vector_.y = (kImageWidth * 0.5 - center_left) * kCorrectionFactor;
		corr_vector_.z = (kImageHeight * 0.5 - center_top) * kCorrectionFactor;
		ROS_INFO("Camera info acquired");
	}
    if(in_reaching_)
    {
        ROS_INFO("Update in cv callback: corr_info %lf %lf", corr_vector_.y, corr_vector_.z);
        SetTargetCorrection();
    }
}

void ArmController::SetTargetCorrection()
{
    if(in_reaching_)
    {
        ROS_INFO("Add correction.");
        // double x, y;
        // if(scanf("%lf %lf", &x, &y) != 2) exit(0);
        // target_end_point_.y += x;
        // target_end_point_.z += y;
        target_end_point_.y = object_end_point_.y + corr_vector_.y;
        target_end_point_.z = object_end_point_.z + corr_vector_.z;
        need_start_ = true;
    }
}

geometry_msgs::Point ArmController::CalculateEndPostion(const KDL::JntArray angle)
{
	KDL::ChainFkSolverPos_recursive fksolver(chain_);
	KDL::Frame pos;
	fksolver.JntToCart(angle, pos);
	geometry_msgs::Point end_point;
	end_point.x = pos.p.x() / double(kLengthFactor);
	end_point.y = pos.p.y() / double(kLengthFactor);
	end_point.z = pos.p.z() / double(kLengthFactor);
	ROS_INFO("now end position: %6.3lf, %6.3lf, %6.3lf", end_point.x, end_point.y, end_point.z);
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
	for(int i = 0; i < kNumJoint - 1; i++)
	{
		if(joint_angle(i) > arm_info_.max_angles[i] ||
				joint_angle(i) < arm_info_.min_angles[i])
		{
			// ROS_WARN("Angle out of bound for joint %d", i);
			is_legal = false;
		}
	}
	double angle3 = M_PI / 2 - joint_angle(2) - joint_angle(1);
	if(angle3 > arm_info_.max_angles[3] ||
			angle3 < arm_info_.min_angles[3])
	{
		// ROS_WARN("Angle out of bound for joint 3");
		is_legal = false;
	}
	return is_legal;
}

}
}
