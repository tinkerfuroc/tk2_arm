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
const double SimpleArmController::kShoulderMoveStep = 0.05;

const double SimpleArmController::kForwardVelocity = 0.05;
const double SimpleArmController::kBlindDistance = 0.05;

const double SimpleArmController::kBaseHeightMin = 0.0;
const double SimpleArmController::kBaseHeightMax = 0.3;
const double SimpleArmController::kBaseHeightDiff = 0.1; 
const double SimpleArmController::kHandLength = 0.12;

const double SEG_MIN[]  =
{
	-94 / 180.0 * M_PI,
	-4  / 180.0 * M_PI,
	45  / 180.0 * M_PI,
	-78 / 180.0 * M_PI
};  //min angle pos

const double SEG_MAX[]  =
{
	32  / 180.0 * M_PI,
	93  / 180.0 * M_PI,
	150 / 180.0 * M_PI,
	59  / 180.0 * M_PI
};  //max angle pos

const double SEG_INIT[] =
{
	-94 / 180.0 * M_PI,
	-4  / 180.0 * M_PI,
	135 / 180.0 * M_PI,
	0   / 180.0 * M_PI
};  //init angle pos

SimpleArmController::SimpleArmController(std::string server_name_) : 
	as_(nh_, "arm_reach_position", boost::bind(&SimpleArmController::PositionCallback, this, _1), false),
	as_init_(nh_, "arm_reset", boost::bind(&SimpleArmController::InitCallback, this, _1), false),
	rate_(10), current_joint_angles_(kNumJoint - 1), target_joint_angles_(kNumJoint - 1), 
	in_grasp_(false), current_height_(0.0), in_init_(false)
{

	base_pub_ = nh_.advertise<std_msgs::Float64>("/base_joint_position_controller/command", 0);
	shoulder_rotation_pub_ = nh_.advertise<std_msgs::Float64>("/shoulder_rotation_joint_position_controller/command", 0);
	shoulder_flexion_pub_ = nh_.advertise<std_msgs::Float64>("/shoulder_flexion_joint_position_controller/command", 0);
	elbow_pub_ = nh_.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 0);
	wrist_deviation_pub_ = nh_.advertise<std_msgs::Float64>("/wrist_deviation_controller/command", 0);
    wrist_extension_pub_ = nh_.advertise<std_msgs::Float64>("/wrist_extension_controller/command", 0);
	hand_pub_ = nh_.advertise<std_msgs::Float64>("/claw_controller/command", 0);
	
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
								   Frame(Vector(0.0, 0.0, 0.40 * double(SimpleArmController::kLengthFactor))));
	arm_info_.segments[2] = Segment(Joint(Joint::RotY),
								   Frame(Vector(0.0, 0.0, 0.40 * double(SimpleArmController::kLengthFactor))));

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
    TurnShoulder();
}

void SimpleArmController::PositionCallback(const tk_arm::ArmReachObjectGoalConstPtr &new_goal)
{
	ROS_ASSERT(new_goal->grasp_state <= 3 && new_goal->grasp_state >= 0);
	double xy_length = sqrt(new_goal->pos.x * new_goal->pos.x + new_goal->pos.y * new_goal->pos.y);
	object_end_point_.x = new_goal->pos.x * (xy_length - kHandLength) / xy_length;
	object_end_point_.y = new_goal->pos.y * (xy_length - kHandLength) / xy_length;
	object_end_point_.z = new_goal->pos.z;
	need_grasp_ = new_goal->grasp_state;

    ROS_INFO("New Goal! [%4.2lf %4.2lf %4.2lf]:%s", object_end_point_.x, object_end_point_.y, object_end_point_.z, 
    	need_grasp_ == 2 ? "release" : need_grasp_ == 1 ? "grasp" : "keep_state");
    if (HasArrivedObject())
    {
		ROS_INFO("No move needed.");
		switch (need_grasp_)
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
		switch (need_grasp_)
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
	double maxDegree = 0.0;
	int interpolationNum = 0.0;
	target_end_point_.x = 0.3;
	target_end_point_.y = 0.0;
	PositionToAngle(target_end_point_, target_joint_angles_);
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
		current_joint_angles_(0) = (target_joint_angles_(0) * i + initial_joint_angles_(0) * (interpolationNum - i)) / interpolationNum;
	    current_joint_angles_(1) = (target_joint_angles_(1) * i + initial_joint_angles_(1) * (interpolationNum - i)) / interpolationNum;
		current_joint_angles_(2) = (target_joint_angles_(2) * i + initial_joint_angles_(2) * (interpolationNum - i)) / interpolationNum;

	    MoveArm();
	    ros::spinOnce();
	    rate_.sleep();
	}
	current_joint_angles_(0) = target_joint_angles_(0);
	current_joint_angles_(1) = target_joint_angles_(1);
	current_joint_angles_(2) = target_joint_angles_(2);
    MoveArm();

	target_height_ = kBaseHeightMin;
	MoveBase();
	// set goal to init angles
	for(int i = 0; i < kNumJoint - 1; i++)
	{
		target_joint_angles_(i) = arm_info_.init_angles[i];
	}
	target_end_point_ = AngleToPosition(target_joint_angles_);

	// interpolation in motor angle
	maxDegree = 0.0;
	interpolationNum = 0.0;
	for (int i = 0; i < 3; ++i)
	{
	    if (fabs(target_joint_angles_(i) - current_joint_angles_(i)) > maxDegree)
	        maxDegree = fabs(target_joint_angles_(i) - current_joint_angles_(i));
	}
	interpolationNum = maxDegree / kDegreeInterpolation + 1;
    initial_joint_angles_ = current_joint_angles_;
	for (int i = 0; i < interpolationNum; ++i)
	{
		current_joint_angles_(0) = (target_joint_angles_(0) * i + initial_joint_angles_(0) * (interpolationNum - i)) / interpolationNum;
	    current_joint_angles_(1) = (target_joint_angles_(1) * i + initial_joint_angles_(1) * (interpolationNum - i)) / interpolationNum;
		current_joint_angles_(2) = (target_joint_angles_(2) * i + initial_joint_angles_(2) * (interpolationNum - i)) / interpolationNum;

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

bool SimpleArmController::GoToPosition()
{
	// interpolate on current-to-object direction	
    ROS_INFO("Go to position [%4.2lf %4.2lf %4.2lf]", object_end_point_.x, object_end_point_.y, object_end_point_.z);
	if (in_init_) TurnShoulder();
	bool is_ok = true;
    std::vector<double> msg;
    msg.resize(kNumJoint);
    MoveBase();
	ROS_INFO("Go to position [%4.2lf %4.2lf %4.2lf] from [%4.2lf %4.2lf %4.2lf]...", 
		object_end_point_.x, object_end_point_.y, object_end_point_.z,
		current_end_point_.x, current_end_point_.y, current_end_point_.z);
	while (!HasArrivedObject())
	{
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
	    MoveArm();
	    ros::spinOnce();
	    rate_.sleep();
	}	
	ROS_INFO("Go to position succedded.");
	return is_ok;
}
void SimpleArmController::TurnShoulder()
{
	while(fabs(current_joint_angles_(0)) > 0.05)
	{
		current_joint_angles_(0) += kShoulderMoveStep;
		MoveArm();
		ros::spinOnce();
		rate_.sleep();
	}
	in_init_ = false;
}

bool SimpleArmController::MoveBase()
{	
	std_msgs::Float64 msg;
	target_height_ = std::max(std::min(object_end_point_.z - kBaseHeightDiff, kBaseHeightMax), kBaseHeightMin);
	bool direction = target_height_ > current_height_;
	ROS_INFO("Move base to %5.2lf. Current base height: %5.2lf. %s.", target_height_, current_height_, direction ? "Moving upwards." : (target_height_ < current_height_ ? "Moving downwards." : "Not moving."));

    msg.data = target_height_;
	base_pub_.publish(msg);
    ros::Duration(100*fabs(target_height_ - current_height_)+2).sleep();
    current_height_ = target_height_;
	ROS_INFO("\033[0;35mBase moved to %5.2lf.\033[0;0m", current_height_);

	object_end_point_.z = object_end_point_.z - target_height_;
	ROS_INFO("Move base succedded.");
	return true;
}

void SimpleArmController::MoveArm()
{
	ROS_INFO("\033[0;34mPublishing angle: %5.2lf %5.2lf %5.2lf %5.2lf\033[0;0m", current_joint_angles_(0)*180/M_PI, (M_PI/2-current_joint_angles_(1))*180/M_PI, 
		(current_joint_angles_(2))*180/M_PI, (M_PI / 2 + current_joint_angles_(1) + current_joint_angles_(2))*180/M_PI);
	current_end_point_ = AngleToPosition(current_joint_angles_);
	std_msgs::Float64 msg;
	msg.data = current_joint_angles_(0);
	shoulder_rotation_pub_.publish(msg);
	msg.data = M_PI/2 - current_joint_angles_(1);
	shoulder_flexion_pub_.publish(msg);
	msg.data = current_joint_angles_(2);
	elbow_pub_.publish(msg);
    msg.data = M_PI;
    wrist_deviation_pub_.publish(msg);
	msg.data = M_PI / 2 + current_joint_angles_(1) + current_joint_angles_(2);
	wrist_extension_pub_.publish(msg);
	msg.data = in_grasp_ ? 0.4 : 1.2;
	hand_pub_.publish(msg);
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
		MoveArm();
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
		MoveArm();
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
