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
const double SimpleArmController::kDegreeInterpolation = (1.0/180.0*M_PI);

const double SimpleArmController::kForwardVelocity = 0.05;
const double SimpleArmController::kBlindDistance = 0.05;

const double SEG_MIN[]  =
{
	-77.1   / 180.0 * M_PI,
	-0.05235988,
	0.52359878,
	-50.0  / 180.0 * M_PI
};  //min angle pos

const double SEG_MAX[]  =
{
	44.1    / 180.0 * M_PI,
	1.6406095,
	2.3500000,
	50 / 180.0 * M_PI
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
	as_init_(nh_, server_name_ + "1", boost::bind(&SimpleArmController::InitCallback, this, _1), false),
	current_joint_angles_(kNumJoint - 1), target_joint_angles_(kNumJoint - 1), in_grasp_(false)
{
	// load arm parameter into arm_info_
	arm_info_.min_angles.resize(SimpleArmController::kNumJoint);
	arm_info_.max_angles.resize(SimpleArmController::kNumJoint);
	arm_info_.init_angles.resize(SimpleArmController::kNumJoint);
	arm_info_.segments.resize(SimpleArmController::kNumSegment);
	for(int i = 0; i < SimpleArmController::kNumJoint; i++)
	{
		arm_info_.min_angles[i] = SEG_MIN[i];
		arm_info_.max_angles[i] = SEG_MAX[i];
		arm_info_.init_angles[i] = SEG_INIT[i];
	}

	arm_info_.segments[0] = Segment(Joint(Joint::RotZ),
								   Frame(Vector(0.0, 0.0, 0.01 * double(SimpleArmController::kLengthFactor))));
	arm_info_.segments[1] = Segment(Joint(Joint::RotY),
								   Frame(Vector(0.0, 0.0, 0.35 * double(SimpleArmController::kLengthFactor))));
	arm_info_.segments[2] = Segment(Joint(Joint::RotY),
								   Frame(Vector(0.0, 0.0, 0.28 * double(SimpleArmController::kLengthFactor))));

	// build the arm model: chain_
	for(int i = 0; i < kNumSegment; i++)
		chain_.addSegment(arm_info_.segments[i]);

	for (int i = 0; i < kNumJoint - 1; ++i)
	{
		current_joint_angles_(i) = SEG_INIT[i];
		target_joint_angles_(i) = SEG_INIT[i];
	}

	ROS_INFO("Arm chain model set. %d segments.", chain_.getNrOfJoints());

	current_end_point_ = AngleToPosition(current_joint_angles_);
	ROS_INFO("Parameters set.");

	//start actionlib servers
	as_.start();
	as_init_.start();
	ROS_INFO("Server started.");
}

void SimpleArmController::PositionCallback(const tk_arm::ArmReachObjectGoalConstPtr &new_goal)
{
	ROS_ASSERT(new_goal->grasp_state <= 3 && new_goal->grasp_state >= 0);
    ROS_INFO("New Goal! [%4.2lf %4.2lf %4.2lf]:%s", new_goal->pos.x, new_goal->pos.y, new_goal->pos.z, 
    	new_goal->grasp_state == 2 ? "release" : new_goal->grasp_state == 1 ? "grasp" : "keep_state");

	object_end_point_ = new_goal->pos;
	need_grasp_ = new_goal->grasp_state;
    if (HasArrivedObject())
    {
		ROS_INFO("No move needed.");
		switch (new_goal->grasp_state)
		{
			case 1: GraspObject(); break;
			case 2: ReleaseObject(); break;
			default: break;
		}
		result_.is_reached = true;
		as_.setSucceeded(result_);		
		ROS_INFO("Action Succeded.");
    }
    else
    {
		result_.is_reached = GoToPosition();
		if (!result_.is_reached)
		{
			ROS_INFO("Go to position failed.");
			as_.setAborted(result_);
		ROS_INFO("Action Aborted.");
			return;
		}
		ROS_INFO("Go to position succeded.");
		switch (new_goal->grasp_state)
		{
			case 1: GraspObject(); break;
			case 2: ReleaseObject(); break;
			default: break;
		}
		as_.setSucceeded(result_);
		ROS_INFO("Action Succeded.");
	}
}

void SimpleArmController::InitCallback(const tk_arm::ArmInitGoalConstPtr &new_goal)
{
	ROS_INFO("Go Init! [%d]", new_goal->state);
	result_init_.is_reached = GoInit();
	ROS_INFO("%s", result_init_.is_reached ? "Go init success." : "Go init failed.");
	as_init_.setSucceeded(result_init_);
}

bool SimpleArmController::GoInit()
{
	// set goal to init angles
	for(int i = 0; i < kNumJoint - 1; i++)
	{
		target_joint_angles_(i) = arm_info_.init_angles[i];
	}
	target_end_point_ = AngleToPosition(target_joint_angles_);

	// interpolation in motor angle
	double maxDegree = 0.0;
	int interpolationNum = 0.0;
	for (int i = 0; i < 3; ++i)
	{
	    if (fabs(target_joint_angles_(i) - current_joint_angles_(i)) > maxDegree)
	        maxDegree = fabs(target_joint_angles_(i) - current_joint_angles_(i));
	}
	interpolationNum = maxDegree / kDegreeInterpolation + 1;
    std::vector<double> msg;
    msg.resize(kNumJoint);
    KDL::JntArray initial_joint_angles_(current_joint_angles_);
	for (int i = 0; i < interpolationNum; ++i)
	{
		ros::Rate rate(10);
	    current_joint_angles_(0) = (target_joint_angles_(0) * i + initial_joint_angles_(0) * (interpolationNum - i)) / interpolationNum;
	    current_joint_angles_(1) = (target_joint_angles_(1) * i + initial_joint_angles_(1) * (interpolationNum - i)) / interpolationNum;
		current_joint_angles_(2) = (target_joint_angles_(2) * i + initial_joint_angles_(2) * (interpolationNum - i)) / interpolationNum;

	    PublishCurrentPose();
	    ros::spinOnce();
	    rate.sleep();
	}
	current_joint_angles_(0) = target_joint_angles_(0);
	current_joint_angles_(1) = target_joint_angles_(1);
	current_joint_angles_(2) = target_joint_angles_(2);
    PublishCurrentPose();	

	return true;
}

bool SimpleArmController::GoToPosition()
{
	// interpolate on current-to-object direction
	bool is_ok = true;
    std::vector<double> msg;
    msg.resize(kNumJoint);
	while (!HasArrivedObject())
	{
		ros::Rate rate(10);
		double x = object_end_point_.x - current_end_point_.x;
		double y = object_end_point_.y - current_end_point_.y;
		double z = object_end_point_.z - current_end_point_.z;
		double distance = sqrt(x * x + y * y + z * z);
		target_end_point_.x = current_end_point_.x + x * kMoveStep / distance;
		target_end_point_.y = current_end_point_.y + y * kMoveStep / distance;
		target_end_point_.z = current_end_point_.z + z * kMoveStep / distance;
		if (!PositionToAngle(target_end_point_, target_joint_angles_))
		{
			ROS_WARN("Position to angle failed.");
			is_ok = false;
			break;
		}

	    current_joint_angles_ = target_joint_angles_;
	    PublishCurrentPose();
	    ros::spinOnce();
	    rate.sleep();
	}
	return is_ok;
}

void SimpleArmController::PublishCurrentPose()
{
	ROS_INFO("\033[0;34mPublishing angle: %5.2lf %5.2lf %5.2lf\033[0;0m", current_joint_angles_(0)*180/M_PI, current_joint_angles_(1)*180/M_PI, 
		current_joint_angles_(2)*180/M_PI);
	current_end_point_ = AngleToPosition(current_joint_angles_);
	// tk_arm::OutPos msg;
	// msg.pos1 = current_joint_angles_(0);
	// msg.pos2 = current_joint_angles_(1);
	// msg.pos3 = current_joint_angles_(2);
	// msg.pos4 = M_PI / 2 - msg.pos2 - msg.pos3;
	// msg.pos5 = 0;
	// msg.pos6 = in_grasp_;
	// position_pub_.publish(msg);
}

bool SimpleArmController::PositionToAngle(const geometry_msgs::Point &target, KDL::JntArray &joint_angle)
{
	Eigen::Matrix<double, 2*(kNumJoint - 1), 1> constraint;
	for(int i = 0; i < kNumJoint - 1; i++)
	{
		constraint(i) = 1;
		constraint(i + kNumJoint - 1) = kAngleFactor;
	}
	KDL::ChainIkSolverPos_LMA solver(chain_, constraint);
	KDL::JntArray new_joint_angles(kNumJoint - 1);
	KDL::JntArray initial_joint_angles_ = current_joint_angles_;
	int retval = -1;
	KDL::Frame pos_goal(KDL::Vector(target.x * kLengthFactor,
									target.y * kLengthFactor,
									target.z * kLengthFactor));
	for(int i = 0; i < kErrorRetry; i++)
	{
		retval = solver.CartToJnt(initial_joint_angles_, pos_goal, new_joint_angles);
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
		initial_joint_angles_.data.setRandom();
		initial_joint_angles_.data *= M_PI;
	}
	ROS_ERROR("failed to calculate joint angle");
	ROS_ERROR("target position %5.2lf, %5.2lf, %5.2lf", target.x, target.y, target.z);
	ROS_INFO("now position %5.2lf, %5.2lf, %5.2lf", current_end_point_.x, current_end_point_.y, current_end_point_.z);
	return false;
}

bool SimpleArmController::HasArrivedObject()
{
	error_.x = current_end_point_.x - object_end_point_.x;
	error_.y = current_end_point_.y - object_end_point_.y;
	error_.z = current_end_point_.z - object_end_point_.z;
	return sqrt(error_.x * error_.x + error_.y  * error_.y  + error_.z  * error_.z ) < 0.01;
}

bool SimpleArmController::HasArrivedTarget()
{
	error_.x = current_end_point_.x - target_end_point_.x;
	error_.y = current_end_point_.y - target_end_point_.y;
	error_.z = current_end_point_.z - target_end_point_.z;
	return sqrt(error_.x * error_.x + error_.y  * error_.y  + error_.z  * error_.z ) < 0.01;
}

bool SimpleArmController::GraspObject()
{
	if (!in_grasp_)
	{
		ros::Duration(kGraspWaitTime).sleep();
		in_grasp_ = true;
		PublishCurrentPose();
		ROS_INFO("\033[1;35mObject grasped!\033[0;0m");
		ros::Duration(kGraspWaitTime).sleep();
		return true;
	}
	return true;
}

bool SimpleArmController::ReleaseObject()
{
	if (in_grasp_)
	{
		ros::Duration(kGraspWaitTime).sleep();
		in_grasp_ = false;
		PublishCurrentPose();
		ROS_INFO("\033[1;35mObject released!\033[0;0m");
		ros::Duration(kGraspWaitTime).sleep();
		return true;
	}
	return true;
}

geometry_msgs::Point SimpleArmController::AngleToPosition(const KDL::JntArray angle)
{
	KDL::ChainFkSolverPos_recursive fksolver(chain_);
	KDL::Frame pos;
	fksolver.JntToCart(angle, pos);
	geometry_msgs::Point end_point;
	end_point.x = pos.p.x() / double(kLengthFactor);
	end_point.y = pos.p.y() / double(kLengthFactor);
	end_point.z = pos.p.z() / double(kLengthFactor);
	ROS_INFO("\033[0;36mCurrent Position: %5.2lf %5.2lf %5.2lf\033[0;0m", end_point.x, end_point.y, end_point.z);
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
