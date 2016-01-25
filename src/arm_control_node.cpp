#include <tk_arm/arm_controller.h>
#include <ros/ros.h>
#include <cmath>
#include <cstdio>

using namespace tinker::arm;
using KDL::Joint;
using KDL::Segment;
using KDL::Frame;
using KDL::Vector;

const double SEG_MIN[]  = {
                    -77.1	/ 180.0 * M_PI,
                    26.9	/ 180.0 * M_PI,
                    24.9	/ 180.0 * M_PI,
                    -77.1	/ 180.0 * M_PI
                    };	//min angle pos

const double SEG_MAX[]  = {
                    44.1	/ 180.0 * M_PI,
                    98.1	/ 180.0 * M_PI,
                    113.1	/ 180.0 * M_PI,
                    3.1	/ 180.0 * M_PI
                    };	//max angle pos

const double SEG_INIT[] = {
                    0		/ 180.0 * M_PI,
                    67		/ 180.0 * M_PI,
                    70		/ 180.0 * M_PI,
                    -30	/ 180.0 * M_PI
                    };	//init angle pos


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle n;
    ArmInfo arm_info;
    arm_info.min_angles.resize(ArmController::kNumJoint);
    arm_info.max_angles.resize(ArmController::kNumJoint);
    arm_info.init_angles.resize(ArmController::kNumJoint);
    arm_info.segments.resize(ArmController::kNumSegment);
    for(int i = 0; i < ArmController::kNumJoint; i++)
    {
        arm_info.min_angles[i] = SEG_MIN[i];
        arm_info.max_angles[i] = SEG_MAX[i];
        arm_info.init_angles[i] = SEG_INIT[i];
    }

    arm_info.segments[0] = Segment(Joint(Joint::RotZ), 
            Frame(Vector(0.0, 0.0, 0.01 * double(ArmController::kLengthFactor))));	
    arm_info.segments[1] = Segment(Joint(Joint::RotY), 
            Frame(Vector(0.0, 0.0, 0.35 * double(ArmController::kLengthFactor))));	
    arm_info.segments[2] = Segment(Joint(Joint::RotY), 
            Frame(Vector(0.0, 0.0, 0.28 * double(ArmController::kLengthFactor))));	
    ArmController arm_controller(arm_info);
    ros::Rate loop_rate(10);
    geometry_msgs::Point point;
    printf("input x, y, z\n");
    if(scanf("%lf %lf %lf", &point.x, &point.y, &point.z) != 3) return 0;
    arm_controller.SetTarget(point);
    while(ros::ok())
    {
        arm_controller.TimeCallback();
        loop_rate.sleep();
    }
    return 0;
}

