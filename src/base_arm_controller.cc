#include <tk_arm/base_arm_controller.h>
#include <ros/ros.h>

namespace tinker
{
namespace arm
{

bool BaseArmController::TimeCallback()
{
    if(is_moving_)
    {
        geometry_msgs::Point new_target;
        KDL::JntArray new_joint_angle;
        if(GetNewTarget(new_target) &
                CalculateJointAngle(new_target, new_joint_angle))
        {
            SetJointAngle(new_joint_angle);
        }
        if(HasArrivedTarget())
        {
            OnArrive();
            is_moving_ = false;
        }
    }
    else
    {
        is_moving_ = NeedStart();
    }
}

}
}
