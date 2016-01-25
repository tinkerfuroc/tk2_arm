#ifndef __TINKER_BASE_ARM_CONTROLLER_H__
#define __TINKER_BASE_ARM_CONTROLLER_H__

#include <vector>
#include <kdl/frames_io.hpp>
#include <geometry_msgs/Point.h>

namespace tinker
{
namespace arm
{
class BaseArmController
{
public:
    BaseArmController()
        :is_moving_(false)
    { }

    virtual bool TimeCallback();

    virtual ~BaseArmController() { }
protected:
    virtual bool NeedStart() = 0;
    virtual bool GetNewTarget(geometry_msgs::Point &new_target) = 0;
    virtual void SetJointAngle(const KDL::JntArray &joint_angle) = 0;
    virtual bool CalculateJointAngle(const geometry_msgs::Point &target, KDL::JntArray &joint_angle) = 0;
    virtual bool HasArrivedTarget() = 0;
    virtual void OnArrive() = 0;

    bool is_moving_;
};
}
}

#endif

