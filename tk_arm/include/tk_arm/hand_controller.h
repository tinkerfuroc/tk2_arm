#ifndef __TINKER_HAND_CONTROLLER_H__
#define __TINKER_HAND_CONTROLLER_H__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tk_arm/ArmHandAction.h>

namespace tinker {
namespace arm {
class HandController {
public:
    HandController(std::string server_name_);

    void HandCallback(const tk_arm::ArmHandGoalConstPtr &goal);

protected:

    virtual bool GraspObject();
    virtual bool ReleaseObject();
    virtual void MoveHand();

    ros::NodeHandle nh_;
    ros::Rate rate_;
    ros::Publisher hand_pub_;

    // actionlib servers
    actionlib::SimpleActionServer<tk_arm::ArmHandAction> as_;
    tk_arm::ArmHandResult result_;

    bool in_grasp_;

private:
    static const int kGraspWaitTime;
};
}
}

#endif
