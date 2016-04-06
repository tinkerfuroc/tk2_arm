#include "ros/ros.h"
#include "tk_arm/OutPos.h"
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <cstdio>
#include <iostream>
// #include <sstream>

#define LENGTH_FAC	(10)
#define ANG_FAC	(0)
#define RATE (10)
#define ERR_ITER (10)
#define INTPOL (1.0/180.0*M_PI)

double SEG_MIN[]  = { -77.1	/ 180.0 * M_PI,
                      26.9	/ 180.0 * M_PI,
                      24.9	/ 180.0 * M_PI,
                      -77.1	/ 180.0 * M_PI
                    };	//min angle pos

double SEG_MAX[]  = {44.1	/ 180.0 * M_PI,
                     98.1	/ 180.0 * M_PI,
                     113.1	/ 180.0 * M_PI,
                     3.1	/ 180.0 * M_PI
                    };	//max angle pos

double SEG_INIT[] = {0		/ 180.0 * M_PI,
                     30		/ 180.0 * M_PI,
                     90		/ 180.0 * M_PI,
                     -30	/ 180.0 * M_PI
                    };	//init angle pos

using namespace KDL;

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
        ROS_INFO("Solution out of range. %d%d%d. POS: %lf %lf %lf. Recalculating...\n", \
                 label[0], label[1], label[2], \
                 q(0) / M_PI * 180.0, q(1) / M_PI * 180.0, q(2) / M_PI * 180.0);
    }
    return isOut;
}

void msg_publish(JntArray q, int times, ros::Publisher pub, ros::Rate rate)
{
    tk_arm::OutPos msg;

    msg.pos1 = q(0);
    msg.pos2 = q(1);
    msg.pos3 = q(2);
    msg.pos4 = M_PI / 2 - msg.pos2 - msg.pos3;
    msg.pos5 = 0;
    msg.pos6 = 0;

    ROS_INFO("OUTPOS:%lf %lf %lf %lf %lf %lf", msg.pos1 / M_PI * 180.0, msg.pos2 / M_PI * 180.0, \
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
        printf("Coordinate %d: ", i);
        if(scanf("%lf", &posinput) != 1)
        {
            exit(0);
        }
        pos_vec(i) = posinput * LENGTH_FAC;
    }
    pos.p = pos_vec;
}

void chain_init(Chain &chain)
{
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.01 * LENGTH_FAC))));	//	-77 degree to 44 degree
    chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(0.0, 0.0, 0.35 * LENGTH_FAC))));	// 27 degree to 98 degree
    chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(0.0, 0.0, 0.28 * LENGTH_FAC))));	// 25 degree to 113 degree
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
        ROS_INFO("Max Iteration Reached. No Solution.");
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

            ROS_INFO("OUTPOS:%lf %lf %lf %lf %lf %lf", msg.pos1 / M_PI * 180.0, msg.pos2 / M_PI * 180.0, \
                     msg.pos3 / M_PI * 180.0, msg.pos4 / M_PI * 180.0, msg.pos5 / M_PI * 180.0, msg.pos6 / M_PI * 180.0);
            ros::spinOnce();

            pub.publish(msg);
            rate.sleep();
        }

        //final publish
        msg_publish(q_sol, 1, pub, rate);

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
    ros::Rate loop_rate(RATE);

    //chain definition
    Chain chain;
    chain_init(chain);

    int n = chain.getNrOfJoints();
    Eigen::Matrix<double, 6, 1> L;
    L(0) = L(1) = L(2) = 1;
    L(3) = L(4) = L(5) = ANG_FAC;
    Frame pos_goal;
    ChainIkSolverPos_LMA solver(chain, L);
    JntArray q(n), q_init(n), q_init2(n), q_sol(n), q_intpol(n);

    arm_init(chain, q_init);
    msg_publish(q_init, 5, position_pub, loop_rate);
    print_pos(chain, q_init, pos_goal);

    while(ros::ok())
    {
        // pos input
        get_pos(pos_goal);

        //solve IK
        int retval;
        retval = solver.CartToJnt(q_init, pos_goal, q_sol);

        //check if solution is valid
        solver_judge(chain, L, retval, q_init, q_sol, pos_goal, position_pub, loop_rate);
    }

    return 0;
}

