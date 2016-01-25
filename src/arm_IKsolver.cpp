#include "ros/ros.h"
#include "tk_arm/OutPos.h"
#include "tk_arm/TargetFound.h"
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <cstdio>
#include <iostream>

static const double LENGTH_FAC = 10.0;
static const double ANG_FAC = 0.0;
static const int RATE = 10;
static const int ERR_ITER = 10;
static const double INTPOL = 1.0 / 180.0 * M_PI;
static const double IMAGE_WIDTH = 640.0;
static const double IMAGE_HEIGHT = 480.0;
static const double DPOS_FAC = 0.0001 * LENGTH_FAC;
static const double FORWARD_VELO = 0.02;
static const double SLEEP_TIME = 0.1;
static const double CATCH_SLEEP_TIME = 0.3;
static const double BLIND_DIST = 0.1;

static const double SEG_MIN[] =
{
    -77.1   / 180.0 * M_PI,
    26.9    / 180.0 * M_PI,
    24.9    / 180.0 * M_PI,
    -77.1   / 180.0 * M_PI
};  //min angle pos

double SEG_MAX[] =
{
    44.1    / 180.0 * M_PI,
    98.1    / 180.0 * M_PI,
    113.1   / 180.0 * M_PI,
    3.1     / 180.0 * M_PI
};  //max angle pos

double SEG_INIT[] =
{
    0       / 180.0 * M_PI,
    40      / 180.0 * M_PI,
    100     / 180.0 * M_PI,
    -50     / 180.0 * M_PI
};  //init angle pos

using namespace KDL;
Frame dpos_new;

bool isAngleOutOfRange(const JntArray q)
{
    bool isOut = false;
    int label[3] = {0};
    for (int i = 0; i < 3; ++i)
    {
        if (q(i) > SEG_MAX[i] || q(i) < SEG_MIN[i])
        {
            isOut = true;
            label[i] = 1;
        }
    }
    if (isOut)
    {
        ROS_INFO("Solution out of range. %d%d%d. POS: %lf %lf %lf. Recalculating...", \
                 label[0], label[1], label[2], \
                 q(0) / M_PI * 180.0, q(1) / M_PI * 180.0, q(2) / M_PI * 180.0);
    }
    return isOut;
}

void update_dpos(const tk_arm::TargetFound& msg)
{
    if (msg.object_recognized) {
        double center_left = msg.object_rect_left + msg.object_rect_width * 0.5;
        double center_top = msg.object_rect_top + msg.object_rect_height * 0.5;
        ROS_INFO("place: %lf %lf", center_left, center_top);
        dpos_new.p(1) = (IMAGE_WIDTH / 2 - center_left) * DPOS_FAC;
        dpos_new.p(2) = (IMAGE_HEIGHT / 2 - center_top) * DPOS_FAC;
        ROS_INFO("Camera info acquired. Correction: %6.3lf %6.3lf.", dpos_new.p(1), dpos_new.p(2));
    }
}

void msg_publish(const JntArray q, const int times, ros::Publisher pub, ros::Rate rate, bool clawcatch)
{
    tk_arm::OutPos msg;

    msg.pos1 = q(0);
    msg.pos2 = q(1);
    msg.pos3 = q(2);
    msg.pos4 = M_PI / 2 - msg.pos2 - msg.pos3;
    msg.pos5 = 0;
    msg.pos6 = clawcatch ? 1 : 0;

    ROS_INFO("OUTPOS:%6.3lf %6.3lf %6.3lf %6.3lf %6.3lf %6.3lf", msg.pos1 / M_PI * 180.0, msg.pos2 / M_PI * 180.0, \
             msg.pos3 / M_PI * 180.0, msg.pos4 / M_PI * 180.0, msg.pos5 / M_PI * 180.0, msg.pos6 / M_PI * 180.0);
    ros::spinOnce();

    for (int i = 0; i < times; ++i)
    {
        pub.publish(msg);
        rate.sleep();
    }
}

void print_pos(const Chain chain, const JntArray q, Frame &pos)
{
    ChainFkSolverPos_recursive fksolver(chain);
    fksolver.JntToCart(q, pos);
    std::cout << "Current Pos: " << pos.p / LENGTH_FAC << std::endl;
}

void get_pos(Frame &pos)
{
    Vector pos_vec;
    for (int i = 0; i < 3 && ros::ok(); ++i)
    {
        double posinput;
        printf("Coord %d: ", i);
        scanf("%lf", &posinput);
        pos_vec(i) = posinput * LENGTH_FAC;
    }
    pos.p = pos_vec;
}

void chain_init(Chain &chain)
{
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.01 * LENGTH_FAC))));  //  -77 degree to 44 degree
    chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(0.0, 0.0, 0.35 * LENGTH_FAC))));  // 27 degree to 98 degree
    chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(0.0, 0.0, 0.28 * LENGTH_FAC))));  // 25 degree to 113 degree
}

void arm_init(const Chain chain, JntArray &q_init)
{
    int n = chain.getNrOfJoints();
    for (int i = 0; i < n; ++i)
    {
        q_init(i) = SEG_INIT[i];
    }
}

int solver_judge(const Chain chain, const Eigen::Matrix<double, 6, 1> L, const int retval, JntArray &q_init, \
                 JntArray &q_sol, Frame &final_pos, ros::Publisher pub, ros::Rate rate)
{
    int errcount = 0, val = retval, return_val = -1;
    int n = chain.getNrOfJoints();
    JntArray q_retry(n);
    ChainIkSolverPos_LMA solver(chain, L);

    double q_sol3 = M_PI / 2 - q_sol(1) - q_sol(2);
    while ((val != 0 || isAngleOutOfRange(q_sol) || q_sol3 > SEG_MAX[3] || q_sol3 < SEG_MIN[3]) && errcount < ERR_ITER)
    {
        if (val != 0)
        {
            switch(val)
            {
            case -1:
                ROS_INFO("Error Code -1. Gradiant too small.");
                break;
            case -2:
                ROS_INFO("Error Code -2. Joint pos increment too small.");
                break;
            case -3:
                ROS_INFO("Error Code -3. Iteration number exceeded.");
                break;
            }
            ROS_INFO("POS: %lf %lf %lf. Recalculating...", q_sol(0) / M_PI * 180.0, q_sol(1) / M_PI * 180.0, q_sol(2) / M_PI * 180.0);
        }
        else if (q_sol3 > SEG_MAX[3] || q_sol3 < SEG_MIN[3])
        {
            ROS_INFO("End rotation angle out of range. Recalculating...");
        }
        q_retry.data.setRandom();
        q_retry.data *= M_PI;
        val = solver.CartToJnt(q_retry, final_pos, q_sol);
        errcount++;
    }

    if (errcount == ERR_ITER)
    {
        ROS_INFO("Max Iteration Reached. No Solution. Cannot reach [%lf, %lf, %lf].", final_pos.p(0) / LENGTH_FAC, \
                 final_pos.p(1) / LENGTH_FAC, final_pos.p(2) / LENGTH_FAC);
        print_pos(chain, q_init, final_pos);
    }
    else
    {
        double maxdegree = 0.0;
        double intpolnum = 0.0;
        for (int i = 0; i < 3; ++i)
        {
            if (fabs(q_sol(i) - q_init(i)) > maxdegree)
                maxdegree = fabs(q_sol(i) - q_init(i));
        }
        intpolnum = maxdegree / double(INTPOL) + 1.0;

        //interpolation publish
        for (int i = 0; i < intpolnum - 1; ++i)
        {
            tk_arm::OutPos msg;

            msg.pos1 = (q_sol(0) * i + q_init(0) * (intpolnum - i)) / intpolnum;
            msg.pos2 = (q_sol(1) * i + q_init(1) * (intpolnum - i)) / intpolnum;
            msg.pos3 = (q_sol(2) * i + q_init(2) * (intpolnum - i)) / intpolnum;
            msg.pos4 = M_PI / 2 - msg.pos2 - msg.pos3;
            msg.pos5 = 0;
            msg.pos6 = 0;

            ROS_INFO("OUTPOS: %6.3lf %6.3lf %6.3lf %6.3lf %6.3lf %6.3lf", msg.pos1 / M_PI * 180.0, msg.pos2 / M_PI * 180.0, \
                     msg.pos3 / M_PI * 180.0, msg.pos4 / M_PI * 180.0, msg.pos5 / M_PI * 180.0, msg.pos6 / M_PI * 180.0);
            ros::spinOnce();

            pub.publish(msg);
            rate.sleep();
        }

        //final publish
        msg_publish(q_sol, 1, pub, rate, 0);

        //print current pos
        print_pos(chain, q_sol, final_pos);

        q_init = q_sol;
        return_val = 0;
    }
    return return_val;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "outpos");
    ros::NodeHandle nodehandle;
    ros::Publisher position_pub = nodehandle.advertise<tk_arm::OutPos>("arm_pos", 0);
    ros::Subscriber position_sub = nodehandle.subscribe("tk2_vision/arm_target_finder/caught_obj", 10, update_dpos);
    ros::Rate loop_rate(RATE);

    //chain definition
    Chain chain;
    chain_init(chain);

    int n = chain.getNrOfJoints();
    double obj_dep;
    Eigen::Matrix<double, 6, 1> L;
    L(0) = L(1) = L(2) = 1;
    L(3) = L(4) = L(5) = ANG_FAC;
    ChainIkSolverPos_LMA solver(chain, L);
    JntArray q_init(n), q_sol(n), q_catch(n);
    Frame pos_goal, pos_goal_corrected;

    arm_init(chain, q_init);
    msg_publish(q_init, 5, position_pub, loop_rate, 0);
    print_pos(chain, q_init, pos_goal);

    dpos_new.p.x(0);
    dpos_new.p.y(0);
    dpos_new.p.z(0);

    printf("Object depth: ");
    scanf("%lf", &obj_dep);
    obj_dep *= LENGTH_FAC;

    while(ros::ok())
    {
        // pos input
        // get_pos(pos_goal);
        if (pos_goal.p(0) < obj_dep - BLIND_DIST) {
            pos_goal.p(0) += FORWARD_VELO * LENGTH_FAC;
            pos_goal_corrected.p = pos_goal.p + dpos_new.p;
        }
        else if (pos_goal.p(0) < obj_dep) {
            pos_goal.p(0) += FORWARD_VELO * LENGTH_FAC;
            pos_goal_corrected.p = pos_goal.p;
        }
        else {
            break;
        }

        ROS_INFO("Coord after correction: [%6.3lf, %6.3lf, %6.3lf].", pos_goal_corrected.p(0) / LENGTH_FAC, \
                 pos_goal_corrected.p(1) / LENGTH_FAC, pos_goal_corrected.p(2) / LENGTH_FAC);

        //solve IK
        int retval;
        retval = solver.CartToJnt(q_init, pos_goal_corrected, q_sol);

        //check if solution is valid
        solver_judge(chain, L, retval, q_init, q_sol, pos_goal_corrected, position_pub, loop_rate);
        pos_goal = pos_goal_corrected;
        ros::spinOnce();
        ros::Duration(SLEEP_TIME).sleep();
    }

    q_catch = q_sol;

    //catch
    ros::Duration(CATCH_SLEEP_TIME).sleep();
    msg_publish(q_init, 1, position_pub, loop_rate, 1);
    ros::Duration(CATCH_SLEEP_TIME).sleep();

    //go back to init
    arm_init(chain, q_init);
    print_pos(chain, q_init, pos_goal);

    int retval;
    retval = solver.CartToJnt(q_catch, pos_goal, q_init);
    solver_judge(chain, L, retval, q_catch, q_init, pos_goal, position_pub, loop_rate);

    return 0;
}

