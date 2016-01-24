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
#define ERR_ITER 50

double SEG_MIN[]	=	{-60.1/180.0*M_PI,27.1/180.0*M_PI,25.1/180.0*M_PI,-77.1/180.0*M_PI};	//max angle pos
double SEG_MAX[]	=	{60.1/180.0*M_PI,98.1/180.0*M_PI,113.1/180.0*M_PI,3.1/180.0*M_PI};	//min angle pos
double SEG_INIT[]	=	{0/180.0*M_PI,67/180.0*M_PI,70/180.0*M_PI,-30/180.0*M_PI};	//init angle pos

using namespace KDL;

bool isAngleOutOfRange(const JntArray q) {
	bool isOut = false;
	for (int i = 0; i < 4; ++i) {
		if (q(i) > SEG_MAX[i] || q(i) < SEG_MIN[i])
			isOut = true;
	}
	if (isOut) {		
		ROS_INFO("Solution out of range.\n");
	}
	return isOut;
}

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
	for (int i = 0; i < n; ++i)
	{
		q_init(i) = SEG_INIT[i]*M_PI/180;
	}

	// angle input
	for(unsigned int i = 0; i < n; i++){
		double myinput;
		printf ("pos of joint %i: ",i);
		scanf ("%lf",&myinput);		
		if (myinput == 999.0) {
			exit(0);
		}
		else{
			myinput = myinput*M_PI/180;
			if (myinput> SEG_MAX[i] || myinput < SEG_MIN[i]) {
				ROS_INFO("Input angle out of range. %lf not between %lf and %lf.\n", \
					myinput/M_PI*180, SEG_MIN[i]/M_PI*180, SEG_MAX[i]/M_PI*180);
				exit(0);
			}
			q(i) = myinput;
		}
	}
	
	fksolver.JntToCart(q,pos_goal);

	int retval;
	retval = solver.CartToJnt(q_init,pos_goal,q_sol);

	// evaluating IKsolver
	while ((retval != 0 || isAngleOutOfRange(q_sol)) && errcount < ERR_ITER) {	
		ROS_INFO("Error Code %d. POS: %lf %lf %lf %lf. Recalculating...\n", retval, q_sol(0)/M_PI*180.0, \
			q_sol(1)/M_PI*180.0, q_sol(2)/M_PI*180.0, q_sol(3)/M_PI*180.0);
		q_init.data.setRandom();
		q_init.data *= M_PI;
		retval = solver.CartToJnt(q_init,pos_goal,q_sol);
		errcount++;
	}

	if (errcount == ERR_ITER) {
		ROS_INFO("Max Iteration Reached. No Solution.\n");
		exit(0);
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

		//send message
		while(ros::ok())
		{
		    position_pub.publish(msg);
		    loop_rate.sleep();
		}
	}
	errcount = 0;

	return 0;
}

