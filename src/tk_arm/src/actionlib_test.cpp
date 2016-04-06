#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tk_arm/ArmReachObjectAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_reach_for_point");
  actionlib::SimpleActionClient<tk_arm::ArmReachObjectAction> ac("tk_arm", true);
  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  tk_arm::ArmReachObjectGoal goal;
  std::cout << "x,y,z of the goal:" << std::endl;
  std::cin >> goal.pos.x >> goal.pos.y >> goal.pos.z;
  std::cout << "grasp state (1 to grasp, 2 to release, 0 to do nothing):" << std::endl;
  std::cin >> goal.grasp_state;
  ac.sendGoal(goal);

  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  return 0;
}
