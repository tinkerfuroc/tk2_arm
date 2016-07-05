#include <tk_arm/arm_ik.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <cstdio>
#include <ros/ros.h>

using KDL::Joint;
using KDL::Segment;
using KDL::Frame;
using KDL::Vector;

namespace tinker {
namespace arm {

const int ArmIK::kNumJoint = 4;
const int ArmIK::kNumMode = 4;
const int ArmIK::kNumSegment = 3;
const int ArmIK::kErrorRetry = 10;

// Length Factor is used for KDL library for better solve result
const double ArmIK::kLengthFactor = 10.0;
const double ArmIK::kAngleFactor = 0.0;
const double ArmIK::kBaseHeightMin = 0.0;
const double ArmIK::kBaseHeightMax = 0.3;
const double ArmIK::kBaseHeightDiff = 0;
const double ArmIK::kHandLength = 0.1;

const double ArmIK::SEG_MIN[] = {-94 / 180.0 * M_PI, -4 / 180.0 * M_PI,
                                 45 / 180.0 * M_PI,
                                 -78 / 180.0 * M_PI};  // min angle pos

const double ArmIK::SEG_MAX[] = {32 / 180.0 * M_PI, 93 / 180.0 * M_PI,
                                 150 / 180.0 * M_PI,
                                 80 / 180.0 * M_PI};  // max angle pos

const double ArmIK::SEG_INIT[] = {-94 / 180.0 * M_PI, -3 / 180.0 * M_PI,
                                  135 / 180.0 * M_PI,
                                  0 / 180.0 * M_PI};  // init angle pos

const double ArmIK::SEG_READY[] = {0 / 180.0 * M_PI, 25 / 180.0 * M_PI,
                                  135 / 180.0 * M_PI,
                                  0 / 180.0 * M_PI};  // init angle pos

const double ArmIK::SEG_KINECT[] = {-55 / 180.0 * M_PI, 90 / 180.0 * M_PI,
                                  90 / 180.0 * M_PI,
                                  50 / 180.0 * M_PI};  // init angle pos

const double ArmIK::SEG_RETRACT[] = {-35 / 180.0 * M_PI, 90 / 180.0 * M_PI,
                                  90 / 180.0 * M_PI,
                                  50 / 180.0 * M_PI};  // init angle pos

ArmIK::ArmIK() {
    // load arm parameter into arm_info_
    arm_info_.min_angles.resize(kNumJoint);
    arm_info_.max_angles.resize(kNumJoint);
    arm_info_.mode_angles.resize(kNumMode, std::vector<double>(kNumJoint, 0));
    arm_info_.segments.resize(kNumSegment);
    for (int i = 0; i < kNumJoint; i++) {
        arm_info_.min_angles[i] = SEG_MIN[i];
        arm_info_.max_angles[i] = SEG_MAX[i];
        arm_info_.mode_angles[0][i] = SEG_INIT[i];
        arm_info_.mode_angles[1][i] = SEG_READY[i];
        arm_info_.mode_angles[2][i] = SEG_KINECT[i];
        arm_info_.mode_angles[3][i] = SEG_RETRACT[i];
    }

    arm_info_.segments[0] =
        Segment(Joint(Joint::RotZ),
                Frame(Vector(0.0, 0.0, 0.01 * double(kLengthFactor))));
    arm_info_.segments[1] =
        Segment(Joint(Joint::RotY),
                Frame(Vector(0.0, 0.0, 0.41 * double(kLengthFactor))));
    arm_info_.segments[2] =
        Segment(Joint(Joint::RotY),
                Frame(Vector(0.0, 0.0, 0.41 * double(kLengthFactor))));

    // build the arm model: chain_
    for (int i = 0; i < kNumSegment; i++)
        chain_.addSegment(arm_info_.segments[i]);
}

bool ArmIK::PositionToAngle(const geometry_msgs::Point &target,
                            KDL::JntArray &joint_angle) {
    KDL::JntArray initial_joint_angles_(ArmIK::kNumJoint - 1);
    initial_joint_angles_.data.setRandom();
    for (int i = 0; i < 3; i ++) {
        initial_joint_angles_.data(i) = SEG_INIT[i];
    }
    return PositionToAngle(target, initial_joint_angles_, joint_angle,
                           kErrorRetry);
}

bool ArmIK::PositionToAngle(const geometry_msgs::Point &target,
                            KDL::JntArray &joint_angle,
                            int retry_time) {
    KDL::JntArray initial_joint_angles_(ArmIK::kNumJoint - 1);
    for (int i = 0; i < 3; i ++) {
        initial_joint_angles_.data(i) = SEG_INIT[i];
    }
    return PositionToAngle(target, initial_joint_angles_, joint_angle,
                           retry_time);
}

bool ArmIK::PositionToAngle(const geometry_msgs::Point &target,
                            const KDL::JntArray &seed_angle,
                            KDL::JntArray &joint_angle, int retry_time) {
    geometry_msgs::Point arm_target;
    double x = target.x;
    double y = target.y;
    double xy_dist = sqrt(x * x + y * y);
    arm_target.x = x - kHandLength * x / xy_dist;
    arm_target.y = y - kHandLength * y / xy_dist;
    arm_target.z = target.z;

    Eigen::Matrix<double, 2 * (kNumJoint - 1), 1> constraint;
    for (int i = 0; i < kNumJoint - 1; i++) {
        constraint(i) = 1;
        constraint(i + kNumJoint - 1) = kAngleFactor;
    }
    KDL::ChainIkSolverPos_LMA solver(chain_, constraint);
    KDL::JntArray initial_joint_angles_ = seed_angle;
    KDL::JntArray new_joint_angles(kNumJoint - 1);
    int retval = -1;
    KDL::Frame pos_goal(KDL::Vector(arm_target.x * kLengthFactor,
                                    arm_target.y * kLengthFactor,
                                    arm_target.z * kLengthFactor));
    for (int i = 0; i < retry_time; i++) {
        retval =
            solver.CartToJnt(initial_joint_angles_, pos_goal, new_joint_angles);
        if (retval != 0) {
            continue;
        }
        if (CheckAngleLegal(new_joint_angles)) {
            joint_angle = new_joint_angles;
            return true;
        }
        initial_joint_angles_.data.setRandom();
        initial_joint_angles_.data *= M_PI;
    }
    return false;
}

geometry_msgs::Point ArmIK::AngleToPosition(const KDL::JntArray angle) {
    KDL::ChainFkSolverPos_recursive fksolver(chain_);
    KDL::Frame pos;
    fksolver.JntToCart(angle, pos);
    geometry_msgs::Point end_point;
    end_point.x = pos.p.x() / double(kLengthFactor);
    end_point.y = pos.p.y() / double(kLengthFactor);
    end_point.z = pos.p.z() / double(kLengthFactor);
    double x = end_point.x;
    double y = end_point.y;
    double xy_dist = sqrt(x * x + y * y);
    end_point.x = x + kHandLength * x / xy_dist;
    end_point.y = y + kHandLength * y / xy_dist;
    return end_point;
}

bool ArmIK::CheckAngleLegal(const KDL::JntArray &joint_angle) {
    bool is_legal = true;
    for (int i = 0; i < kNumJoint - 1; i++) {
        if (joint_angle(i) > arm_info_.max_angles[i] ||
            joint_angle(i) < arm_info_.min_angles[i]) {
            is_legal = false;
        }
    }
    double angle3 = M_PI / 2 - joint_angle(2) - joint_angle(1);
    if (angle3 > arm_info_.max_angles[3] || angle3 < arm_info_.min_angles[3]) {
        is_legal = false;
    }
    return is_legal;
}
}
}
