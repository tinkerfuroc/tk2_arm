#include "ros/ros.h"
#include "tk_arm/OutPos.h"
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <cstdio>
#include <iostream>
#include <sstream>

#define LENGTH_FAC	9
#define ANG_FAC	5

#define SEG0_MIN	-60.1/180.0*M_PI
#define SEG0_MAX	60.1/180.0*M_PI
#define SEG0_INIT	0/180.0*M_PI

#define SEG1_MIN	27.1/180.0*M_PI
#define SEG1_MAX	98.1/180.0*M_PI
#define SEG1_INIT	67/180.0*M_PI

#define SEG2_MIN	25.1/180.0*M_PI
#define SEG2_MAX	113.1/180.0*M_PI
#define SEG2_INIT	70/180.0*M_PI

#define SEG3_MIN	-77.1/180.0*M_PI
#define SEG3_MAX	3.1/180.0*M_PI
#define SEG3_INIT	-30/180.0*M_PI

using namespace KDL;

int main(int argc, char **argv)
{
	Chain chain;
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.01*LENGTH_FAC))));	//
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.35*LENGTH_FAC))));	// 27 degree to 98 degree
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.28*LENGTH_FAC))));	// 25 degree to 113 degree
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.10*LENGTH_FAC))));	// -77 degree to 3 degree
	// chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.0))));
	
	int n = chain.getNrOfJoints();
	Eigen::Matrix<double,6,1> L;
	L(0)=1;L(1)=1;L(2)=1;
	L(3)=L(4)=L(5)=ANG_FAC;
	Frame pos_goal;
	ChainFkSolverPos_recursive fksolver(chain);
	ChainIkSolverPos_LMA solver(chain,L);	
	JntArray q(n), q_init(n), q_sol(n);
	double x, y, z;
	
	ros::init(argc, argv, "outpos");
	ros::NodeHandle nodehandle;
	ros::Publisher position_pub = nodehandle.advertise<tk_arm::OutPos>("arm_pos", 0);
	ros::Rate loop_rate(10);

	int count = 0;
	int errcount = 0;
	q_init(0)=SEG0_INIT*M_PI/180;
	q_init(1)=SEG1_INIT*M_PI/180;
	q_init(2)=SEG2_INIT*M_PI/180;
	q_init(3)=SEG3_INIT*M_PI/180;

	for(unsigned int i = 0; i < n; i++){
		double myinput;
		printf ("pos of joint %i: ",i+1);
		scanf ("%lf",&myinput);
		if (myinput == 999.0) exit(0);
		q(i)=myinput*M_PI/180;
	}
	
	fksolver.JntToCart(q,pos_goal);

	int retval;
	retval = solver.CartToJnt(q_init,pos_goal,q_sol);

	while (!(retval == 0 
		&& q_sol(0) < SEG0_MAX && q_sol(0) > SEG0_MIN
		&& q_sol(1) < SEG1_MAX && q_sol(1) > SEG1_MIN
		&& q_sol(2) < SEG2_MAX && q_sol(2) > SEG2_MIN
		&& q_sol(3) < SEG3_MAX && q_sol(3) > SEG3_MIN) && errcount < 50) {
	
		ROS_INFO("Error Code %d. POS: %lf %lf %lf %lf. Recalculating...\n", retval, q_sol(0)/M_PI*180.0, \
			q_sol(1)/M_PI*180.0, q_sol(2)/M_PI*180.0, q_sol(3)/M_PI*180.0);

		q_init.data.setRandom();
		q_init.data *= M_PI;
		retval = solver.CartToJnt(q_init,pos_goal,q_sol);
		errcount++;
	}

	if (errcount == 500) {
		ROS_INFO("Max Iteration Reached. No Solution.\n");
	}
	else {	
		q_init = q_sol;
		
		tk_arm::OutPos msg;

		msg.pos1 = q_sol(0);
		msg.pos2 = q_sol(1);
		msg.pos3 = q_sol(2);
		msg.pos4 = q_sol(3);
		msg.pos5 = 0;
		msg.pos6 = 0;

		position_pub.publish(msg);
		ROS_INFO("OUTPOS:%lf %lf %lf %lf %lf %lf\n", msg.pos1/M_PI*180.0, msg.pos2/M_PI*180.0, \
			msg.pos3/M_PI*180.0, msg.pos4/M_PI*180.0, msg.pos5/M_PI*180.0, msg.pos6/M_PI*180.0);
		ros::spinOnce();

		while(ros::ok())
		{
		    position_pub.publish(msg);
		    loop_rate.sleep();
		}
	}
	errcount = 0;

	return 0;
}

