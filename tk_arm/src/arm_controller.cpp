#include <tk_arm/arm_controller.h>
#include <tk_arm/arm_ik.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <std_msgs/Float64.h>
#include <cmath>
#include <assert.h>
#include <ros/ros.h>

using namespace tinker::arm;
using KDL::Joint;
using KDL::Segment;
using KDL::Frame;
using KDL::Vector;

const double ArmController::kMoveStep = 0.01;
const double ArmController::kDegreeInterpolation = (2.0 / 180.0 * M_PI);
const double ArmController::kShoulderMoveStep = 0.05;
const double ArmController::kKinectAngle = 3.4; 

namespace tinker {
namespace arm {
ArmController::ArmController(std::string server_name_)
    : nh_(),
      as_(nh_, "arm_reach_position",
          boost::bind(&ArmController::PositionCallback, this, _1), false),
      as_init_(nh_, "arm_reset",
               boost::bind(&ArmController::ModeCallback, this, _1), false),
      as_path_(nh_, "arm_path",
               boost::bind(&ArmController::PathCallback, this, _1), false),
      rate_(10),
      current_joint_angles_(ArmIK::kNumJoint - 1),
      target_joint_angles_(ArmIK::kNumJoint - 1),
      initial_joint_angles_(ArmIK::kNumJoint - 1),
      mode_joint_angles_(ArmIK::kNumJoint - 1),
      current_height_(0.0), mode_height_(0.0),
      in_init_(false), in_mode_(false),in_kinect_(false)
{
    base_pub_ = nh_.advertise<std_msgs::Float64>(
        "/base_joint_position_controller/command", 0);
    shoulder_rotation_pub_ = nh_.advertise<std_msgs::Float64>(
        "/shoulder_rotation_joint_position_controller/command", 0);
    shoulder_flexion_pub_ = nh_.advertise<std_msgs::Float64>(
        "/shoulder_flexion_joint_position_controller/command", 0);
    elbow_pub_ = nh_.advertise<std_msgs::Float64>(
        "/elbow_joint_position_controller/command", 0);
    wrist_deviation_pub_ = nh_.advertise<std_msgs::Float64>(
        "/wrist_deviation_controller/command", 0);
    wrist_extension_pub_ = nh_.advertise<std_msgs::Float64>(
        "/wrist_extension_controller/command", 0);
    ROS_INFO("Publisher started.");

    for (int i = 0; i < (ArmIK::kNumJoint - 1); ++i) {
        current_joint_angles_(i) = ArmIK::SEG_INIT[i];
        target_joint_angles_(i) = ArmIK::SEG_INIT[i];
        initial_joint_angles_(i) = ArmIK::SEG_INIT[i];
    }

    GoMode(1);
    mode_joint_angles_ = current_joint_angles_;
    // TurnShoulder();
    // current_end_point_ = arm_ik_.AngleToPosition(current_joint_angles_);

    // start actionlib servers
    as_.start();
    ROS_INFO("Action Server 'arm_reach_position' started.");
    as_init_.start();
    ROS_INFO("Action Server 'arm_reset' started.");
    as_path_.start();
    ROS_INFO("Action Server 'arm_path' started.");
}

void ArmController::PositionCallback(
    const tk_arm::ArmReachObjectGoalConstPtr &new_goal) {
    if (new_goal->pos.header.frame_id != "arm_origin_link") {
        ROS_WARN("Forcing frame id to be arm_origin_link");
    }

    if (in_init_) {
        GoMode(1);
        mode_joint_angles_ = current_joint_angles_;
        in_init_ = false;
        in_mode_ = false;
    }
    if (in_mode_){
        GoToLastPathEnd();
    }

    geometry_msgs::Point origin_current_end_point_ = current_end_point_;
    KDL::JntArray origin_current_joint_angles_ = current_joint_angles_;

    double x = new_goal->pos.point.x;
    double y = new_goal->pos.point.y;
    double z = new_goal->pos.point.z;
    object_end_point_.x = x;
    object_end_point_.y = y;
    object_end_point_.z = z;

    bool test_state = new_goal->state & 0x04;
    bool success = true;

    // ROS_INFO("New Goal! \033[1;34m[%4.2lf %4.2lf %4.2lf]\033[0;0m", object_end_point_.x,
    //          object_end_point_.y, object_end_point_.z);
    if (!HasArrivedObject()) {
         ROS_INFO("Go to \033[1;34m[%4.2lf %4.2lf %4.2lf]\033[0;0m", object_end_point_.x,
         object_end_point_.y, object_end_point_.z);
        result_.is_reached = GoToPosition(!test_state);
        if (!result_.is_reached) {
            success = false;
        }
    }
    result_.moved = current_end_point_;
    result_.is_reached = success;
    if (success) {
        as_.setSucceeded(result_);
        ROS_INFO("\033[1;31mAction Succeeded.\033[0;0m");
    } else {
        as_.setAborted(result_);
        ROS_INFO("\033[1;31mAction Aborted.\033[0;0m");
    }

    if (test_state) {
        current_end_point_ = origin_current_end_point_;
        current_joint_angles_ = origin_current_joint_angles_;
        //ROS_INFO("\033[0;31mTEST STATE ENDS\033[0;0m");
    }
}

void ArmController::ModeCallback(
    const tk_arm::ArmInitGoalConstPtr &new_goal) {
    switch(new_goal->state){
        case 0: ROS_INFO("Go Init!"); break;
        case 1: ROS_INFO("Go Ready!"); break;
        case 2: ROS_INFO("Go Dodge Kinect!"); break;
        default: break;
    }
    result_init_.is_reached = GoMode(new_goal->state);
    ROS_INFO("%s",
             result_init_.is_reached ? "Go to mode success." : "Go to mode failed.");
    as_init_.setSucceeded(result_init_);
}

void ArmController::PathCallback(
    const tk_arm::ArmPathGoalConstPtr &new_goal) {
    if (new_goal->path.header.frame_id != "arm_origin_link") {
        ROS_WARN("Forcing frame id %s to be arm_origin_link",
                new_goal->path.header.frame_id.c_str());
    }

    if (in_init_) {
        GoMode(1);
        mode_joint_angles_ = current_joint_angles_;
        in_init_ = false;
        in_mode_ = false;
    }
    if (in_mode_){
        GoToLastPathEnd();
    }

    bool success = true;
    bool cancelled = false;
    for (int i = 0; i < new_goal->path.poses.size(); ++i) {
        if (as_path_.isPreemptRequested()) {
            ROS_INFO("[Arm controller] Cancel requested!");
            cancelled = true;
            break;
        }
        double x = new_goal->path.poses[i].pose.position.x;
        double y = new_goal->path.poses[i].pose.position.y;
        double z = new_goal->path.poses[i].pose.position.z;
        object_end_point_.x = x;
        object_end_point_.y = y;
        object_end_point_.z = z;

        ROS_INFO("New Goal! \033[1;34m[%4.2lf %4.2lf %4.2lf]\033[0;0m", object_end_point_.x, object_end_point_.y, object_end_point_.z);
        // ROS_INFO("Go to position \033[1;34m[%4.2lf %4.2lf %4.2lf]\033[0;0m", object_end_point_.x,
             // object_end_point_.y, object_end_point_.z);
        if (!HasArrivedObject()) {
            result_path_.is_reached = GoToPosition(true);
            if (!result_path_.is_reached) {
                ROS_INFO("Go to position failed.");
                success = false;
            }
        }
    }

    result_path_.moved = current_end_point_;
    result_path_.is_reached = success;
    if (cancelled) {
        as_path_.setPreempted(result_path_);
        ROS_INFO("\033[1;32mAction Cancelled.\033[0;0m");
    } else if (success) {
        as_path_.setSucceeded(result_path_);
        ROS_INFO("\033[1;32mAction Succeded.\033[0;0m");
    } else {
        as_path_.setAborted(result_path_);
        ROS_INFO("\033[1;31mAction Aborted.\033[0;0m");
    }
}

bool ArmController::GoToLastPathEnd() {

    ROS_INFO("GO TO LAST PATH END");
    target_height_ = mode_height_;
    MoveBase(1);
    // set goal to init angles
    target_joint_angles_ = mode_joint_angles_;

    // interpolation in motor angle
    double maxDegree = 0.0;
    for (int i = 0; i < 3; ++i) {
        if (fabs(target_joint_angles_(i) - current_joint_angles_(i)) > maxDegree)
            maxDegree = fabs(target_joint_angles_(i) - current_joint_angles_(i));
    }
    int interpolationNum = maxDegree / kDegreeInterpolation + 1;
    KDL::JntArray initial_angles_(current_joint_angles_);
    for (int i = 0; i < interpolationNum; ++i) {
        current_joint_angles_(0) =
            (target_joint_angles_(0) * i +
             initial_angles_(0) * (interpolationNum - i)) /
            interpolationNum;
        current_joint_angles_(1) =
            (target_joint_angles_(1) * i +
             initial_angles_(1) * (interpolationNum - i)) /
            interpolationNum;
        current_joint_angles_(2) =
            (target_joint_angles_(2) * i +
             initial_angles_(2) * (interpolationNum - i)) /
            interpolationNum;
        MoveArm();
        ros::spinOnce();
        rate_.sleep();
    }
    current_joint_angles_(0) = target_joint_angles_(0);
    current_joint_angles_(1) = target_joint_angles_(1);
    current_joint_angles_(2) = target_joint_angles_(2);
    MoveArm();
    current_end_point_ = arm_ik_.AngleToPosition(current_joint_angles_);
    current_end_point_.z += current_height_;
    
    in_mode_ = false; 
    //ros::Duration(4).sleep();
    return true;
}

bool ArmController::GoMode(int mode) {
    ROS_INFO("GO MODE");
    target_height_ = ArmIK::kBaseHeightMin;
    mode_height_ = current_height_;
    MoveBase(1);
    // set goal to init angles
    for (int i = 0; i < ArmIK::kNumJoint - 1; i++) {
        switch(mode){
            case 0: target_joint_angles_(i) = ArmIK::SEG_INIT[i]; break;
            case 1: target_joint_angles_(i) = ArmIK::SEG_READY[i]; break;
            case 2: target_joint_angles_(i) = ArmIK::SEG_KINECT[i]; break;
            case 3: target_joint_angles_(i) = ArmIK::SEG_RETRACT[i]; break;
            default: break;
        }        
    }
    // target_joint_angles_(1) = ArmIK::SEG_MIN[1];
    target_end_point_ = arm_ik_.AngleToPosition(target_joint_angles_);

   // interpolation in motor angle
    double maxDegree = 0.0;
    for (int i = 0; i < 3; ++i) {
        if (fabs(target_joint_angles_(i) - current_joint_angles_(i)) > maxDegree)
            maxDegree = fabs(target_joint_angles_(i) - current_joint_angles_(i));
    }
    int interpolationNum = maxDegree / kDegreeInterpolation + 1;
    mode_joint_angles_ = current_joint_angles_;
    for (int i = 0; i < interpolationNum; ++i) {
        current_joint_angles_(0) =
            (target_joint_angles_(0) * i +
             mode_joint_angles_(0) * (interpolationNum - i)) /
            interpolationNum;
        current_joint_angles_(1) =
            (target_joint_angles_(1) * i +
             mode_joint_angles_(1) * (interpolationNum - i)) /
            interpolationNum;
        current_joint_angles_(2) =
            (target_joint_angles_(2) * i +
             mode_joint_angles_(2) * (interpolationNum - i)) /
            interpolationNum;

        MoveArm();
        ros::spinOnce();
        rate_.sleep();
    }
    current_joint_angles_(0) = target_joint_angles_(0);
    current_joint_angles_(1) = target_joint_angles_(1);
    current_joint_angles_(2) = target_joint_angles_(2);
    MoveArm();
    
    if (mode!=1){
        std_msgs::Float64 msg;
        msg.data = kKinectAngle;
        wrist_extension_pub_.publish(msg);
    }

    current_end_point_ = arm_ik_.AngleToPosition(current_joint_angles_);
    if (mode == 0) in_init_ = true;
    in_mode_ = true;
    ROS_INFO("IN INIT: %d, IN MODE: %d", in_init_, in_mode_);
    return true;
}

bool ArmController::GoToPosition(bool move) {
    // interpolate on current-to-object direction

    bool is_ok = true;
    std::vector<double> msg;
    msg.resize(ArmIK::kNumJoint);

    while (!HasArrivedObject()) {
    ROS_INFO(
        "Go to \033[1;34m[%4.2lf %4.2lf %4.2lf]\033[0;0m from \033[1;35m[%4.2lf %4.2lf %4.2lf]\033[0;0m...",
        object_end_point_.x, object_end_point_.y, object_end_point_.z,
        current_end_point_.x, current_end_point_.y, current_end_point_.z);
        double diff_x = object_end_point_.x - current_end_point_.x;
        double diff_y = object_end_point_.y - current_end_point_.y;
        double diff_z = object_end_point_.z - current_end_point_.z;
        double distance = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
        target_end_point_.x = current_end_point_.x + diff_x * kMoveStep / distance;
        target_end_point_.y = current_end_point_.y + diff_y * kMoveStep / distance;
        double target_z = current_end_point_.z + diff_z * kMoveStep / distance ;
        if (target_z < ArmIK::kBaseHeightMin + ArmIK::kBaseHeightDiff) {
            target_end_point_.z = target_z - ArmIK::kBaseHeightMin;
            target_height_ = ArmIK::kBaseHeightMin;
        }
        else if (target_z > ArmIK::kBaseHeightMax + ArmIK::kBaseHeightDiff) {
            target_end_point_.z = target_z - ArmIK::kBaseHeightMax;
            target_height_ = ArmIK::kBaseHeightMax;
        }
        else {
            target_height_ = target_z - ArmIK::kBaseHeightDiff;
            target_end_point_.z = ArmIK::kBaseHeightDiff;
        }
        //ROS_INFO("Target Height: %lf, Current Height: %lf", target_height_, current_height_);
        MoveBase(move);

        if (!arm_ik_.PositionToAngle(target_end_point_, current_joint_angles_, target_joint_angles_, 10)) {
            ROS_WARN("Position to angle failed.");
            return false;
        }
        current_joint_angles_ = target_joint_angles_;

        if (move) {
            MoveArm();
            ros::spinOnce();
            rate_.sleep();
        }     
        current_end_point_ = arm_ik_.AngleToPosition(current_joint_angles_);
        current_end_point_.z += target_height_;
        ROS_DEBUG("current at %f %f %f", current_end_point_.x, current_end_point_.y, current_end_point_.z);
    }
    return is_ok;
}

void ArmController::TurnShoulder() {
    while (fabs(current_joint_angles_(0)) > 0.05) {
        current_joint_angles_(0) += kShoulderMoveStep;
        MoveArm();
        ros::spinOnce();
        rate_.sleep();
    }
    in_init_ = false;
    current_end_point_ = arm_ik_.AngleToPosition(current_joint_angles_);
}

bool ArmController::MoveBase(bool move) {
    std_msgs::Float64 msg;
    bool direction = target_height_ > current_height_;
    //ROS_INFO("Move base to \033[1;36m%4.2lf\033[0;0m. Current base height: \033[1;35m%4.2lf\033[0;0m. %s.",
    //         target_height_, current_height_,
    //         direction ? "Moving upwards."
    //                   : (target_height_ < current_height_ ? "Moving downwards."
    //                                                       : "Not moving."));

    if (move) {
        msg.data = target_height_;
        base_pub_.publish(msg);
        // ros::Duration(50 * fabs(target_height_ - current_height_) + 2).sleep();
        current_height_ = target_height_;
        ROS_INFO("\033[0;35mBase moved to m%4.2lf.", current_height_);
    }
    // ROS_INFO("Move base succedded.");
    return true;
}

void ArmController::MoveArm() {
    ROS_INFO("\033[0;34mPub angles: %4.2lf %4.2lf %4.2lf %4.2lf\033[0;0m",
             current_joint_angles_(0) * 180 / M_PI,
             (M_PI / 2 - current_joint_angles_(1)) * 180 / M_PI,
             (current_joint_angles_(2)) * 180 / M_PI,
             (M_PI / 2 + current_joint_angles_(1) + current_joint_angles_(2)) *
                 180 / M_PI);
    std_msgs::Float64 msg;
    msg.data = current_joint_angles_(0);
    shoulder_rotation_pub_.publish(msg);
    msg.data = M_PI / 2 - current_joint_angles_(1);
    shoulder_flexion_pub_.publish(msg);
    msg.data = current_joint_angles_(2);
    elbow_pub_.publish(msg);
    msg.data = M_PI;
    wrist_deviation_pub_.publish(msg);
    msg.data =
        M_PI / 2 + current_joint_angles_(1) + current_joint_angles_(2) + 0.2;
    double min_wrist_angle = M_PI + ArmIK::SEG_MIN[3];
    double max_wrist_angle = M_PI + ArmIK::SEG_MAX[3];
    msg.data = msg.data < min_wrist_angle ? min_wrist_angle : msg.data;
    msg.data = msg.data > max_wrist_angle ? max_wrist_angle : msg.data;
    wrist_extension_pub_.publish(msg);
}

bool ArmController::HasArrivedObject() {
    error_.x = current_end_point_.x - object_end_point_.x;
    error_.y = current_end_point_.y - object_end_point_.y;
    error_.z = current_end_point_.z - object_end_point_.z;
    return sqrt(error_.x * error_.x + error_.y * error_.y +
                error_.z * error_.z) < 0.01;
}

bool ArmController::HasArrivedTarget() {
    error_.x = current_end_point_.x - target_end_point_.x;
    error_.y = current_end_point_.y - target_end_point_.y;
    error_.z = current_end_point_.z - target_end_point_.z;
    return sqrt(error_.x * error_.x + error_.y * error_.y +
                error_.z * error_.z) < 0.01;
}

}
}
