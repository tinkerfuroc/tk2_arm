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

namespace tinker
{
namespace arm
{
struct ArmInfo
{
    std::vector<double> min_angles;
    std::vector<double> max_angles;
    std::vector<double> init_angles;
    std::vector<double> duck_angles;
    std::vector<KDL::Segment> segments;
};

class SimpleArmController
{
public:
    SimpleArmController(std::string server_name_);

    void PositionCallback(const tk_arm::ArmReachObjectGoalConstPtr &goal);

    void GoInit();

    void PublishCurrentPose();

    static const int kNumJoint;
    static const int kNumSegment;
    static const int kErrorRetry;
    static const int kGraspWaitTime;

    static const double kLengthFactor;
    static const double kAngleFactor;
    static const double kMoveStep;    

    static const double kForwardVelocity;
    static const double kBlindDistance;

protected:
    virtual void SetJointAngle(const KDL::JntArray &joint_angle);
    virtual bool CalculateJointAngle(const geometry_msgs::Point &target, KDL::JntArray &joint_angle);
    virtual bool HasArrivedTarget();
    virtual bool GraspObject();
    virtual bool ReleaseObject();

    void GoToPosition();
    void PublishMissionDone();

    geometry_msgs::Point CalculateEndPostion(const KDL::JntArray angle);
    void WarnKDLSolve(int retval);
    bool CheckAngleLegal(const KDL::JntArray &joint_angle);

    ArmInfo arm_info_;
    KDL::Chain chain_;
    KDL::JntArray now_joint_angles_;
    KDL::JntArray target_joint_angles_;
    ros::Publisher position_pub_;
    ros::Publisher mission_pub_;
    ros::Subscriber position_sub_;

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<tk_arm::ArmReachObjectAction> as_;
    tk_arm::ArmReachObjectResult result_;
    
    int need_grasp_; //1 to grasp, 2 to release, 0 to do nothing 
    bool in_grasp_;
    bool error_last_time_;
    bool in_reaching_;
    bool in_retreiving_;
    bool in_init_;

    geometry_msgs::Point now_end_point_;
    geometry_msgs::Point target_end_point_;
    geometry_msgs::Point object_end_point_;
    geometry_msgs::Point error_;
};
}
}

#endif

