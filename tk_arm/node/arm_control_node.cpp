#include <ros/ros.h>
#include <tk_arm/ArmReachObjectAction.h>
#include <actionlib/server/simple_action_server.h>
#include <tk_arm/arm_controller.h>
#include <tk_arm/hand_controller.h>
#include <cmath>
#include <cstdio>

using namespace tinker::arm;


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "tk_arm_control");

	ArmController arm_controller(ros::this_node::getName());
	HandController hand_controller(ros::this_node::getName());
	ros::spin();
	
	return 0;
}

