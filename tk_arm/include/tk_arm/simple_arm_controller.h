#ifndef __TINKER_SIMPLE_ARM_CONTROLLER_H__
#define __TINKER_SIMPLE_ARM_CONTROLLER_H__

#include <ros/ros.h>
#include <vector>
#include <kdl/frames_io.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <actionlib/server/simple_action_server.h>
#include <tk_arm/ArmReachObjectAction.h>
// #include <tk_arm/ArmReachTestAction.h>
#include <tk_arm/ArmInitAction.h>

namespace tinker {
namespace arm {
struct ArmInfo {
    std::vector<double> min_angles;
    std::vector<double> max_angles;
    std::vector<double> init_angles;
    std::vector<KDL::Segment> segments;
};

class SimpleArmController {
public:
    SimpleArmController(std::string server_name_);

    // callback function for GoToPosition actionlib server
    void PositionCallback(const tk_arm::ArmReachObjectGoalConstPtr &goal);

    // callback function for GoInit actionlib server
    void InitCallback(const tk_arm::ArmInitGoalConstPtr &new_goal);

    static const int kNumJoint;
    static const int kNumSegment;
    static const int kErrorRetry;
    static const int kGraspWaitTime;

    static const double kLengthFactor;
    static const double kAngleFactor;
    static const double kMoveStep;
    static const double kDegreeInterpolation;
    static const double kShoulderMoveStep;

    static const double kBaseHeightMax;
    static const double kBaseHeightMin;
    static const double kBaseHeightDiff;

protected:
    virtual bool PositionToAngle(const geometry_msgs::Point &target,
                                 KDL::JntArray &joint_angle);
    virtual geometry_msgs::Point AngleToPosition(const KDL::JntArray angle);

    virtual bool GraspObject();
    virtual bool ReleaseObject();

    virtual void MoveArm();
    virtual bool GoToPosition(bool move);
    virtual bool GoInit();
    virtual void TurnShoulder();
    virtual bool MoveBase(bool move);

    bool HasArrivedTarget();
    bool HasArrivedObject();

    void WarnKDLSolve(int retval);
    bool CheckAngleLegal(const KDL::JntArray &joint_angle);

    ArmInfo arm_info_;  // arm parameters
    KDL::Chain chain_;  // the arm

    // publishers for PublishCurrentPose()
    ros::NodeHandle nh_;
    ros::Rate rate_;
    ros::Publisher base_pub_;
    ros::Publisher shoulder_rotation_pub_;
    ros::Publisher shoulder_flexion_pub_;
    ros::Publisher elbow_pub_;
    ros::Publisher wrist_deviation_pub_;
    ros::Publisher wrist_extension_pub_;
    ros::Publisher hand_pub_;

    // actionlib servers
    actionlib::SimpleActionServer<tk_arm::ArmReachObjectAction> as_;
    actionlib::SimpleActionServer<tk_arm::ArmInitAction> as_init_;
    tk_arm::ArmReachObjectResult result_;
    tk_arm::ArmInitResult result_init_;

    int need_grasp_;  // 1 to grasp, 2 to release, 0 to do nothing
    bool in_grasp_;
    bool in_init_;

    // arm states
    KDL::JntArray current_joint_angles_;
    KDL::JntArray target_joint_angles_;
    double current_height_;
    double target_height_;

    geometry_msgs::Point current_end_point_;
    geometry_msgs::Point target_end_point_;  // TARGET is the next midway-point for arm
    geometry_msgs::Point object_end_point_;  // OBJECT is the final goal for arm
    geometry_msgs::Point error_;
};
}
}

#endif
