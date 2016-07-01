#ifndef __TINKER_ARM_IK_H__
#define __TINKER_ARM_IK_H__

#include <kdl/frames_io.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <geometry_msgs/Point.h>

namespace tinker {
namespace arm {

struct ArmInfo {
    std::vector<double> min_angles;
    std::vector<double> max_angles;
    std::vector<std::vector<double> > mode_angles;
    std::vector<KDL::Segment> segments;
};

class ArmIK {
public:
    ArmIK();

    bool PositionToAngle(const geometry_msgs::Point &target,
                         KDL::JntArray &joint_angle);

    bool PositionToAngle(const geometry_msgs::Point &target,
                         KDL::JntArray &joint_angle,
                         int retry_time = 10);

    bool PositionToAngle(const geometry_msgs::Point &target,
                         const KDL::JntArray &seed_angle,
                         KDL::JntArray &joint_angle,
                         int retry_time = 10);


    geometry_msgs::Point AngleToPosition(const KDL::JntArray angle);

    bool CheckAngleLegal(const KDL::JntArray &joint_angle);

    static const int kNumJoint;
    static const int kNumMode;
    static const int kNumSegment;
    static const int kErrorRetry;

    static const double kLengthFactor;
    static const double kAngleFactor;
    static const double kBaseHeightMax;
    static const double kBaseHeightMin;
    static const double kBaseHeightDiff;
    static const double kHandLength;

    static const double SEG_MIN[];
    static const double SEG_MAX[];
    static const double SEG_INIT[];
    static const double SEG_READY[];
    static const double SEG_KINECT[];
    static const double SEG_RETRACT[];

private:
    void WarnKDLSolve(int retval);
    KDL::Chain chain_;  // the arm
    ArmInfo arm_info_;  // arm parameters
};
}
}

#endif
