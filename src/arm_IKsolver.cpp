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

double SEG_MIN[]  = {-77.1	/180.0*M_PI,
					 26.9	/180.0*M_PI,
					 24.9	/180.0*M_PI,
					 -77.1	/180.0*M_PI};	//min angle pos

double SEG_MAX[]  = {44.1	/180.0*M_PI,
					 98.1	/180.0*M_PI,
					 113.1	/180.0*M_PI,
					 3.1	/180.0*M_PI};	//max angle pos

double SEG_INIT[] = {0		/180.0*M_PI,
					 67		/180.0*M_PI,
					 70		/180.0*M_PI,
					 -30	/180.0*M_PI};	//init angle pos

using namespace KDL;

bool isAngleOutOfRange(const JntArray q) {
	bool isOut = false;
	int label[3]={0};
	for (int i = 0; i < 3; ++i) {
		if (q(i) > SEG_MAX[i] || q(i) < SEG_MIN[i]) {
			isOut = true;
			label[i] = 1;
		}
	}
	if (isOut) {
		ROS_INFO("Solution out of range. %d%d%d. POS: %lf %lf %lf. Recalculating...\n", \
			label[0], label[1], label[2], \
			q(0)/M_PI*180.0, q(1)/M_PI*180.0, q(2)/M_PI*180.0);
	}
	return isOut;
}

int main(int argc, char **argv)
{
	Chain chain;
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.01*LENGTH_FAC))));	//	-77 degree to 44 degree
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0.35*LENGTH_FAC))));	// 27 degree to 98 degree
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0.28*LENGTH_FAC))));	// 25 degree to 113 degree
	
	int n = chain.getNrOfJoints();
	Eigen::Matrix<double,6,1> L;
	L(0)=1;L(1)=1;L(2)=1;
	L(3)=L(4)=L(5)=ANG_FAC;
	Frame pos_goal;
	ChainFkSolverPos_recursive fksolver(chain);
	ChainIkSolverPos_LMA solver(chain,L);	
	JntArray q(n), q_init(n), q_init2(n), q_sol(n), q_intpol(n);
	double x, y, z;
	
	ros::init(argc, argv, "outpos");
	ros::NodeHandle nodehandle;
	ros::Publisher position_pub = nodehandle.advertise<tk_arm::OutPos>("arm_pos", 0);
	ros::Rate loop_rate(RATE);
	tk_arm::OutPos msg;

	int count = 0;
	int errcount = 0;
	for (int i = 0; i < n; ++i)
	{
		q_init(i) = SEG_INIT[i];
	}

	//initiating
	msg.pos1 = q_init(0);
	msg.pos2 = q_init(1);
	msg.pos3 = q_init(2);
	msg.pos4 = M_PI/2 - q_init(1) - q_init(2);
	msg.pos5 = 0;
	msg.pos6 = 0;

	ROS_INFO("OUTPOS:%lf %lf %lf %lf %lf %lf\n", msg.pos1/M_PI*180.0, msg.pos2/M_PI*180.0, \
		msg.pos3/M_PI*180.0, msg.pos4/M_PI*180.0, msg.pos5/M_PI*180.0, msg.pos6/M_PI*180.0);
	ros::spinOnce();

	//send init message
	for (int i = 0; i < 5; ++i)
	{
		position_pub.publish(msg);
		loop_rate.sleep();
	}
	
	//print init position
	fksolver.JntToCart(q_init,pos_goal);
	std::cout << "Current Pos: " << pos_goal.p / LENGTH_FAC << std::endl;

	while(ros::ok())
	{		
		// pos input
		Vector pos_vec;
		for (int i = 0; i < 3 && ros::ok(); ++i)
		{
			double posinput;
			printf("POS %d: ", i);
			scanf("%lf", &posinput);
			pos_vec(i) = posinput * LENGTH_FAC;
		}
		pos_goal.p = pos_vec;

		//solve IK
		int retval;
		retval = solver.CartToJnt(q_init,pos_goal,q_sol);
		double q_sol3 = M_PI/2 - q_sol(1) - q_sol(2);

		// evaluating IKsolver
		while ((retval != 0 || isAngleOutOfRange(q_sol) || q_sol3 > SEG_MAX[3] || q_sol3 < SEG_MIN[3]) \
			&& errcount < ERR_ITER) {
			if (retval != 0) {
				ROS_INFO("Error Code %d. POS: %lf %lf %lf. Recalculating...\n", \
					retval, q_sol(0)/M_PI*180.0, \
					q_sol(1)/M_PI*180.0, q_sol(2)/M_PI*180.0);
			}
			else if (q_sol3 > SEG_MAX[3] || q_sol3 < SEG_MIN[3]) {
				ROS_INFO("End rotation angle out of range. Recalculating...\n");				
			}
			q_init2.data.setRandom();
			q_init2.data *= M_PI;
			retval = solver.CartToJnt(q_init2,pos_goal,q_sol);
			errcount++;
		}

		if (errcount == ERR_ITER) {
			ROS_INFO("Max Iteration Reached. No Solution.\n");
			//print current pos
			fksolver.JntToCart(q_init,pos_goal);
			std::cout << "Current Pos: " << pos_goal.p / LENGTH_FAC << std::endl;
		}
		else {	
			double maxdegree = 0.0;
			double intpolnum = 0.0;
			for (int i = 0; i < 4; ++i)
			{
				if (fabs(q_sol(i)-q_init(i)) > maxdegree)
					maxdegree = fabs(q_sol(i)-q_init(i));
			}
			intpolnum = maxdegree / double(INTPOL) + 1.0;

			//interpolation publish
			for (int i = 0; i < intpolnum - 1; ++i)
			{
				msg.pos1 = (q_sol(0)*i + q_init(0)*(intpolnum - i))/intpolnum;
				msg.pos2 = (q_sol(1)*i + q_init(1)*(intpolnum - i))/intpolnum;
				msg.pos3 = (q_sol(2)*i + q_init(2)*(intpolnum - i))/intpolnum;
				msg.pos4 = M_PI/2 - msg.pos2 - msg.pos3;
				msg.pos5 = 0;
				msg.pos6 = 0;

				ROS_INFO("OUTPOS:%lf %lf %lf %lf %lf %lf\n", msg.pos1/M_PI*180.0, msg.pos2/M_PI*180.0, \
					msg.pos3/M_PI*180.0, msg.pos4/M_PI*180.0, msg.pos5/M_PI*180.0, msg.pos6/M_PI*180.0);
				ros::spinOnce();

				position_pub.publish(msg);
				loop_rate.sleep();
			}

			//final publish
			msg.pos1 = q_sol(0);
			msg.pos2 = q_sol(1);
			msg.pos3 = q_sol(2);
			msg.pos4 = M_PI/2 - q_sol(1) - q_sol(2);
			msg.pos5 = 0;
			msg.pos6 = 0;

			ROS_INFO("OUTPOS:%lf %lf %lf %lf %lf %lf\n", msg.pos1/M_PI*180.0, msg.pos2/M_PI*180.0, \
				msg.pos3/M_PI*180.0, msg.pos4/M_PI*180.0, msg.pos5/M_PI*180.0, msg.pos6/M_PI*180.0);
			ros::spinOnce();

			position_pub.publish(msg);
			loop_rate.sleep();

			//print current pos
			fksolver.JntToCart(q_sol,pos_goal);
			std::cout << "Current Pos: " << pos_goal.p / LENGTH_FAC << std::endl;
						
			q_init = q_sol;	
		}
		errcount = 0;
	}

	return 0;
}

