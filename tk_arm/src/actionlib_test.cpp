#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tk_arm/ArmReachObjectAction.h>
#include <tk_arm/ArmInitAction.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_reach_for_point");
    // clients for GoToPoint and GoToInit action
    actionlib::SimpleActionClient<tk_arm::ArmReachObjectAction> ac(
        "arm_reach_position", true);
    actionlib::SimpleActionClient<tk_arm::ArmInitAction> ac1("arm_reset", true);

    // start the clients
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ac1.waitForServer();
    ROS_INFO("Action server started.");

    // initialize the goals
    tk_arm::ArmReachObjectGoal goal;
    tk_arm::ArmInitGoal goal1;

    goal.pos.header.frame_id = "arm_origin_link";
    while (ros::ok()) {
        int a;
        printf("choose mode (0 for pose, 1 for init):\n");
        if (scanf("%d", &a) != 1) break;
        // client for GoToPoint
        if (a == 0) {
            // input x, y, z and claw behaviour(grasp, release or keep the same)
            printf("x,y,z of the goal:\n");
            if (3 != scanf("%lf%lf%lf", &goal.pos.point.x, &goal.pos.point.y,
                           &goal.pos.point.z))
                break;
            printf("grasp state (0: release, 1: grasp, 2,3: force, 4: test):\n");
            if (1 != scanf("%d", &goal.state)) break;
            while (goal.state > 4 || goal.state < 0) {
                printf("grasp state (0: release, 1: grasp, 2,3: force, 4: test):\n");
                scanf("%d", &goal.state);
            }

            goal.pos.header.stamp = ros::Time::now();

            ROS_INFO("Sending goal...");
            ac.sendGoal(goal);

            ROS_INFO("Waiting for result...");
            bool finished_before_timeout =
                ac.waitForResult();
            if (finished_before_timeout) {
                actionlib::SimpleClientGoalState state = ac.getState();
                ROS_INFO("Action finished: %s", state.toString().c_str());
            } else
                ROS_INFO("Action did not finish before the time out.");
        }
        // client for GoToInit
        else if (a == 1) {
            // the goal is of no use now
            printf("state of the goal:\n");
            if (1 != scanf("%d", &goal1.state)) break;

            ROS_INFO("Sending goal...");
            ac1.sendGoal(goal1);

            ROS_INFO("Waiting for result...");
            bool finished_before_timeout =
                ac1.waitForResult(ros::Duration(30.0));
            if (finished_before_timeout) {
                actionlib::SimpleClientGoalState state = ac1.getState();
                ROS_INFO("Action finished: %s", state.toString().c_str());
            } else
                ROS_INFO("Action did not finish before the time out.");
        } else
            break;
    }

    return 0;
}
