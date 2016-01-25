#ifndef __TINKER_ARM_CONTROLLER_H__
#define __TINKER_ARM_CONTROLLER_H__

#include <vector>
#include <kdl/frames_io.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <geometry_msgs/Point.h>
#include <tk_arm/base_arm_controller.h>
#include <ros/ros.h>

namespace tinker
{
namespace arm
{
struct ArmInfo
{
    std::vector<double> min_angles;
    std::vector<double> max_angles;
    std::vector<double> init_angles;
    std::vector<KDL::Segment> segments;
};

class ArmController:public BaseArmController
{
public:
    ArmController(const ArmInfo &arminfo);

    void SetTarget(const geometry_msgs::Point &point);
    
    void PublishNowPose();

    static const int kNumJoint;
    static const int kNumSegment;
    static const int kLengthFactor;
    static const int kAngleFactor;
    static const int kErrorRetry;
    static const double kMoveStep;
protected:
    virtual bool NeedStart();
    virtual bool GetNewTarget(geometry_msgs::Point &new_target);
    virtual void SetJointAngle(const KDL::JntArray &joint_angle);
    virtual bool CalculateJointAngle(const geometry_msgs::Point &target, KDL::JntArray &joint_angle);
    virtual bool HasArrivedTarget();
    virtual void OnArrive();

    geometry_msgs::Point CalculateEndPostion();
    void WarnKDLSolve(int retval);
    bool CheckAngleLegal(const KDL::JntArray &joint_angle);

    ArmInfo arm_info_;
    KDL::Chain chain_;
    KDL::JntArray now_joint_angles_;
    ros::Publisher position_pub_;
    
    int grasp_wait_time_;
    bool need_start_;

    geometry_msgs::Point now_end_point_;
    geometry_msgs::Point target_end_point_;
    geometry_msgs::Point error_;
};
}
}

#endif

