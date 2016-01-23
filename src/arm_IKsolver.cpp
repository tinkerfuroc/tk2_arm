#include "ros/ros.h"
#include "tk_arm/OutPos.h"
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <cstdio>
#include <iostream>
#include <sstream>

#define factor 9
#define angfac 5

using namespace KDL;

int main(int argc, char **argv)
{
	Chain chain;
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.01*factor))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.35*factor))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.28*factor))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.10*factor))));
	// chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.0))));
	
	int n = chain.getNrOfJoints();
	Eigen::Matrix<double,6,1> L;
	L(0)=1;L(1)=1;L(2)=1;
	L(3)=L(4)=L(5)=angfac;
	ChainFkSolverPos_recursive fksolver(chain);
	ChainIkSolverPos_LMA solver(chain,L);	
	JntArray q(n), q_init(n), q_sol(n);
	double x, y, z;
	
	ros::init(argc, argv, "outpos");
	ros::NodeHandle nodehandle;
	ros::Publisher position_pub = nodehandle.advertise<tk_arm::OutPos>("arm_pos", 0);
	ros::Rate loop_rate(10);

    Frame pos_goal;

    for(unsigned int i = 0; i < n; i++){
        double myinput;
        printf ("pos of joint %i: ",i+1);
        scanf ("%lf",&myinput);
        q(i)=myinput*M_PI/180;
    }
    
    fksolver.JntToCart(q,pos_goal);

    // for (int i = 0; i < 4; ++i)
    // {
    // 	for (int j = 0; j < 4; ++j)
    // 	{
    // 		std::cout << "pos_goal:\n" << pos_goal(i,j) << std::endl;
    // 	}
    // }

    int retval;
    retval = solver.CartToJnt(q_init,pos_goal,q_sol);

    // std::cout << "------------------" << std::endl;
    // std::cout << "ret_val\t" << retval << std::endl;
    // std::cout << "q_init:\t" << q_init.data.transpose()/M_PI*180.0 << std::endl;
    std::cout << "pos_goal:\n" << pos_goal << std::endl;
    // std::cout << "q_sol:\t" << q_sol.data.transpose()/M_PI*180.0 << std::endl;

    tk_arm::OutPos msg;

    msg.pos1 = q_sol(0);
    msg.pos2 = q_sol(1);
    msg.pos3 = q_sol(2);
    msg.pos4 = q_sol(3);
    msg.pos5 = 0;
    msg.pos6 = 0;

    position_pub.publish(msg);
    ROS_INFO("OUTPOS:%lf %lf %lf %lf %lf %lf", msg.pos1/M_PI*180.0, msg.pos2/M_PI*180.0, \
        msg.pos3/M_PI*180.0, msg.pos4/M_PI*180.0, msg.pos5/M_PI*180.0, msg.pos6/M_PI*180.0);
    while(ros::ok())
    {
        position_pub.publish(msg);
        loop_rate.sleep();
    }

	return 0;
}

