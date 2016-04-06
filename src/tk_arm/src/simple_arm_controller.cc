#include "tk_arm/simple_arm_controller.h"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <std_msgs/String.h>
#include <cmath>
#include <assert.h>
#include <ros/ros.h>
#include <iostream>

using namespace tinker::arm;
using KDL::Joint;
using KDL::Segment;
using KDL::Frame;
using KDL::Vector;

namespace tinker
{
namespace arm
{

const int SimpleArmController::kNumJoint = 4;
const int SimpleArmController::kNumSegment = 3;
const int SimpleArmController::kErrorRetry = 10;
const int SimpleArmController::kGraspWaitTime = 1;

//Length Factor is used for KDL library for better solve result
const double SimpleArmController::kLengthFactor = 10.0;
const double SimpleArmController::kAngleFactor = 0.0;
const double SimpleArmController::kMoveStep = 0.005;

const double SimpleArmController::kForwardVelocity = 0.05;
const double SimpleArmController::kBlindDistance = 0.05;

const double SEG_MIN[]  =
{
	-77.1   / 180.0 * M_PI,
	26.9    / 180.0 * M_PI,
	24.9    / 180.0 * M_PI,
	-77.1   / 180.0 * M_PI
};  //min angle pos

const double SEG_MAX[]  =
{
	44.1    / 180.0 * M_PI,
	98.1    / 180.0 * M_PI,
	113.1   / 180.0 * M_PI,
	3.1 / 180.0 * M_PI
};  //max angle pos

const double SEG_INIT[] =
{
	0       / 180.0 * M_PI,
	30      / 180.0 * M_PI,
	100     / 180.0 * M_PI,
	-40     / 180.0 * M_PI
};  //init angle pos

SimpleArmController::SimpleArmController(std::string server_name_) : 
	as_(nh_, server_name_, boost::bind(&SimpleArmController::PositionCallback, this, _1), false),
	// action_name_(server_name_),
	now_joint_angles_(kNumJoint - 1),
	in_grasp_(2), error_last_time_(false),
	in_reaching_(false), in_retreiving_(false),
	in_init_(true)
{
	// arm_info_.min_angles.resize(SimpleArmController::kNumJoint);
	// arm_info_.max_angles.resize(SimpleArmController::kNumJoint);
	// arm_info_.init_angles.resize(SimpleArmController::kNumJoint);
	// arm_info_.segments.resize(SimpleArmController::kNumSegment);
	// for(int i = 0; i < SimpleArmController::kNumJoint; i++)
	// {
	// 	arm_info_.min_angles[i] = SEG_MIN[i];
	// 	arm_info_.max_angles[i] = SEG_MAX[i];
	// 	arm_info_.init_angles[i] = SEG_INIT[i];
	// }

	// arm_info_.segments[0] = Segment(Joint(Joint::RotZ),
	// 							   Frame(Vector(0.0, 0.0, 0.01 * double(SimpleArmController::kLengthFactor))));
	// arm_info_.segments[1] = Segment(Joint(Joint::RotY),
	// 							   Frame(Vector(0.0, 0.0, 0.35 * double(SimpleArmController::kLengthFactor))));
	// arm_info_.segments[2] = Segment(Joint(Joint::RotY),
	// 							   Frame(Vector(0.0, 0.0, 0.28 * double(SimpleArmController::kLengthFactor))));

	// for(int i = 0; i < kNumSegment; i++)
	// {
	// 	chain_.addSegment(arm_info_.segments[i]);
	// }
	ROS_INFO("Parameters set.");
	as_.start();
	// GoInit();
}

void SimpleArmController::PositionCallback(const tk_arm::ArmReachObjectGoalConstPtr &new_goal)
{
    ROS_INFO("New Goal!");
    // if (new_goal->pos.x < 0.1 && new_goal->pos.y < 0.1 && new_goal->pos.z < 0.1)
    //     return;
  //   if (new_goal->grasp_state == 4)
  //   {
	 //    ROS_INFO("Go Init!");
		// KDL::JntArray angles(kNumJoint - 1);
		// for(int i = 0; i < kNumJoint - 1; i++)
		// {
		// 	angles(i) = arm_info_.init_angles[i];
		// }
		// geometry_msgs::Point point = CalculateEndPostion(angles);
		// object_end_point_ = point;
		// result_.is_reached = true;
		// as_.setSucceeded(result_);
  //   	return;
  //   }
	object_end_point_ = new_goal->pos;
	need_grasp_ = new_goal->grasp_state;
	result_.is_reached = true;
	as_.setSucceeded(result_);
}

void SimpleArmController::GoInit()
{
    ROS_INFO("Go Init!");
	KDL::JntArray angles(kNumJoint - 1);
	for(int i = 0; i < kNumJoint - 1; i++)
	{
		angles(i) = arm_info_.init_angles[i];
	}
	geometry_msgs::Point point = CalculateEndPostion(angles);
	object_end_point_ = point;
	GoToPosition();
}

void SimpleArmController::GoToPosition()
{
	result_.is_reached = true;
	as_.setSucceeded(result_);
}

void SimpleArmController::PublishCurrentPose()
{
	// tk_arm::OutPos msg;
	// msg.pos1 = now_joint_angles_(0);
	// msg.pos2 = now_joint_angles_(1);
	// msg.pos3 = now_joint_angles_(2);
	// msg.pos4 = M_PI / 2 - msg.pos2 - msg.pos3;
	// msg.pos5 = 0;
	// msg.pos6 = in_grasp_;
	// position_pub_.publish(msg);
}

void SimpleArmController::PublishMissionDone()
{
 //    std_msgs::String msg;
 //    ROS_ERROR("PublishMissionDone!");
	// msg.data = "Done.";
	// mission_pub_.publish(msg);	
}

// bool SimpleArmController::TimeCallback()
// {
// 	if (error_last_time_)
// 	{
// 		ROS_WARN("Cannot reach");
//         GoInit();
//         need_grasp_ = false;
//         in_reaching_ = false;
//         in_retreiving_ = false;
// 		in_init_ = true;
//         in_duck_ = false;
//         error_last_time_ = false;
// 	}
// 	if (HasArrivedTarget())
// 	{
//         if (in_init_)
// 		{
// 			if(need_grasp_ == true)
// 			{
//                 ROS_ERROR("Need grasp!");
//                 waiting_target_ = false;
// 				in_init_ = false;
// 				in_reaching_ = true;
// 			}
//             else
//             {
//                 if(!waiting_target_)
//                     PublishMissionDone();
//                 waiting_target_ = true;
//             }
//             ROS_WARN("Init target: %lf %lf %lf", object_end_point_.x, object_end_point_.y, object_end_point_.z);
// 		}
// 		else if(in_reaching_)
// 		{
// 			geometry_msgs::Point point = target_end_point_;
// 			if (!aligned_with_object_)
// 			{
//                 double factor = now_end_point_.x / object_end_point_.x;
// 				point.y = object_end_point_.y * factor;
// 				point.z = object_end_point_.z * factor;
// 				SetTarget(point);
// 				aligned_with_object_ = true;
// 			}
// 			else if (point.x < object_end_point_.x)
// 			{
// 				if (point.x < object_end_point_.x - kBlindDistance)
// 				{
// 					point.x += kForwardVelocity;
//                     double factor = point.x / object_end_point_.x;
//                     point.y = object_end_point_.y * factor;
//                     point.z = object_end_point_.z * factor;
// 					SetTarget(point);
// 				}
// 				else
// 				{
// 					printf("correction count: %d\n", correction_updated_count_);
// 					if (correction_updated_count_ >= kCorrectionCount)
// 					{
// 						point.x = object_end_point_.x;
// 						SetTarget(point);
// 					}
// 					else
// 					{
// 						GoInit();
//                         need_grasp_ = false;
// 						in_reaching_ = false;
// 						in_init_ = true;
// 						aligned_with_object_ = false;                            	
// 					}
// 				}
// 			}
// 			else
// 			{
// 				GraspObject();
// 				GoInit();
//                 need_grasp_ = false;
// 				in_reaching_ = false;
// 				in_retreiving_ = true;
// 				aligned_with_object_ = false;
// 			}
// 		}
// 		else if(in_retreiving_)
// 		{
// 			ReleaseObject();
//             GoInit();
//             need_grasp_ = false;
// 			in_retreiving_ = false;
// 			in_init_ = true;
// 		}
// 	}
// 	BaseSimpleArmController::TimeCallback();
// }

// bool SimpleArmController::GetNewTarget(geometry_msgs::Point &new_target)
// {
// 	double x = target_end_point_.x - now_end_point_.x;
// 	double y = target_end_point_.y - now_end_point_.y;
// 	double z = target_end_point_.z - now_end_point_.z;
// 	double distance = sqrt(x * x + y * y + z * z);
// 	new_target.x = now_end_point_.x + x * kMoveStep / distance;
// 	new_target.y = now_end_point_.y + y * kMoveStep / distance;
// 	new_target.z = now_end_point_.z + z * kMoveStep / distance;
// 	return true;
// }

void SimpleArmController::SetJointAngle(const KDL::JntArray &joint_angle)
{
	// need_start_ = false;
	PublishCurrentPose();
	now_joint_angles_ = joint_angle;
	now_end_point_ = CalculateEndPostion(now_joint_angles_);
}

bool SimpleArmController::CalculateJointAngle(const geometry_msgs::Point &target, KDL::JntArray &joint_angle)
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

bool SimpleArmController::HasArrivedTarget()
{
	error_.x = now_end_point_.x - target_end_point_.x;
	error_.y = now_end_point_.y - target_end_point_.y;
	error_.z = now_end_point_.z - target_end_point_.z;
	return sqrt(error_.x * error_.x + error_.y  * error_.y  + error_.z  * error_.z ) < 0.01;
}

bool SimpleArmController::GraspObject()
{
	if (!in_grasp_)
	{
		ros::Duration(kGraspWaitTime).sleep();
		in_grasp_ = true;
		PublishCurrentPose();
		ROS_INFO("Target grasped!");
		ros::Duration(kGraspWaitTime).sleep();
		return true;
	}
	return false;
}

bool SimpleArmController::ReleaseObject()
{
	if (in_grasp_)
	{
		ros::Duration(kGraspWaitTime).sleep();
		in_grasp_ = false;
		PublishCurrentPose();
		ROS_INFO("Target released!");
		ros::Duration(kGraspWaitTime).sleep();
		return true;
	}
	return false;
}

geometry_msgs::Point SimpleArmController::CalculateEndPostion(const KDL::JntArray angle)
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

void SimpleArmController::WarnKDLSolve(int retval)
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

bool SimpleArmController::CheckAngleLegal(const KDL::JntArray &joint_angle)
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
