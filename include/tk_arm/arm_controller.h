#ifndef __TINKER_ARM_CONTROLLER_H__
#define __TINKER_ARM_CONTROLLER_H__

#include "tk_arm/TargetFound.h"
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
    std::vector<double> duck_angles;
    std::vector<KDL::Segment> segments;
};

class ArmController:public BaseArmController
{
public:
    ArmController(const ArmInfo &arminfo);

    void SetTarget(const geometry_msgs::Point &point);

    void SetObject(const geometry_msgs::Point &point);

    void GoInit();

    void GoDuck();

    void PublishNowPose();

    bool TimeCallback();

    static const int kNumJoint;
    static const int kNumSegment;
    static const int kErrorRetry;
    static const double kLengthFactor;
    static const double kAngleFactor;
    static const double kMoveStep;    
    static const double kImageWidth;
    static const double kImageHeight;
    static const double kCorrectionFactor;
    static const int kCorrectionCount;
    static const double kForwardVelocity;
    static const double kBlindDistance;

protected:
    virtual bool NeedStart();
    virtual bool GetNewTarget(geometry_msgs::Point &new_target);
    virtual void SetJointAngle(const KDL::JntArray &joint_angle);
    virtual bool CalculateJointAngle(const geometry_msgs::Point &target, KDL::JntArray &joint_angle);
    virtual bool HasArrivedTarget();
    virtual void OnArriveTarget();
    virtual bool GraspObject();
    virtual bool ReleaseObject();
    virtual void UpdateCVCorrection(const tk_arm::TargetFound& msg);
    virtual void SetTargetCorrection();

    geometry_msgs::Point CalculateEndPostion(const KDL::JntArray angle);
    void WarnKDLSolve(int retval);
    bool CheckAngleLegal(const KDL::JntArray &joint_angle);

    ArmInfo arm_info_;
    KDL::Chain chain_;
    KDL::JntArray now_joint_angles_;
    KDL::JntArray target_joint_angles_;
    ros::Publisher position_pub_;
    ros::Subscriber position_sub_;
    
    int grasp_wait_time_;
    bool need_start_;
    bool in_grasp_;
    bool error_last_time_;
    int correction_updated_count_;
    bool in_reaching_;
    bool in_retreiving_;
    bool in_init_;
    bool in_duck_;
    bool aligned_with_object_;

    geometry_msgs::Point now_end_point_;
    geometry_msgs::Point corr_vector_;
    geometry_msgs::Point target_end_point_;
    geometry_msgs::Point object_end_point_;
    geometry_msgs::Point error_;
};
}
}

#endif

