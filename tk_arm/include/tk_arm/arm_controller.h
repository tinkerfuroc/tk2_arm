#ifndef __TINKER_ARM_CONTROLLER_H__
#define __TINKER_ARM_CONTROLLER_H__

#include <ros/ros.h>
#include <vector>
#include <tk_arm/arm_ik.h>
#include <actionlib/server/simple_action_server.h>
#include <tk_arm/ArmReachObjectAction.h>
#include <tk_arm/ArmInitAction.h>
#include <tk_arm/ArmPathAction.h>

namespace tinker {
namespace arm {
class ArmController {
public:
    ArmController(std::string server_name_);

    // callback function for GoToPosition actionlib server
    void PositionCallback(const tk_arm::ArmReachObjectGoalConstPtr &goal);

    // callback function for GoInit actionlib server
    void ModeCallback(const tk_arm::ArmInitGoalConstPtr &new_goal);
    void PathCallback(const tk_arm::ArmPathGoalConstPtr &new_goal);

protected:
    virtual void MoveArm();
    virtual bool GoToPosition(bool move);
    virtual bool GoMode(int mode);
    virtual bool GoToLastPathEnd();
    virtual void TurnShoulder();
    virtual bool MoveBase(bool move);

    bool HasArrivedTarget();
    bool HasArrivedObject();

    // publishers for PublishCurrentPose()
    ros::NodeHandle nh_;
    ros::Rate rate_;
    ros::Publisher base_pub_;
    ros::Publisher shoulder_rotation_pub_;
    ros::Publisher shoulder_flexion_pub_;
    ros::Publisher elbow_pub_;
    ros::Publisher wrist_deviation_pub_;
    ros::Publisher wrist_extension_pub_;

    // actionlib servers
    actionlib::SimpleActionServer<tk_arm::ArmReachObjectAction> as_;
    actionlib::SimpleActionServer<tk_arm::ArmInitAction> as_init_;
    actionlib::SimpleActionServer<tk_arm::ArmPathAction> as_path_;
    tk_arm::ArmReachObjectResult result_;
    tk_arm::ArmInitResult result_init_;
    tk_arm::ArmPathResult result_path_;

    bool in_init_;
    bool in_mode_;
    bool in_kinect_;

    // arm states
    KDL::JntArray current_joint_angles_;
    KDL::JntArray target_joint_angles_;
    KDL::JntArray initial_joint_angles_;
    KDL::JntArray mode_joint_angles_;
    double mode_height_;
    double current_height_;
    double target_height_;

    geometry_msgs::Point current_end_point_;
    geometry_msgs::Point target_end_point_;  // TARGET is the next midway-point for arm
    geometry_msgs::Point object_end_point_;  // OBJECT is the final goal for arm
    geometry_msgs::Point error_;
private:
    ArmIK arm_ik_;
    static const int kGraspWaitTime;
    static const double kMoveStep;
    static const double kDegreeInterpolation;
    static const double kShoulderMoveStep;
    static const double kKinectAngle;
};
}
}

#endif
