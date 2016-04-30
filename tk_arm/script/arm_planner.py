#!/usr/bin/python

import actionlib
import rospy
from tk_arm.msg._ArmReachObjectGoal import ArmReachObjectGoal
from tk_arm.msg._ArmReachObjectFeedback import ArmReachObjectFeedback
from tk_arm.msg._ArmReachObjectResult import ArmReachObjectResult
from tk_arm.msg._ArmReachObjectAction import ArmReachObjectAction
from tinker_msgs.msg._SimpleMoveGoal import SimpleMoveGoal
from tinker_msgs.msg._SimpleMoveAction import SimpleMoveAction
import tf
from math import fabs

class ArmPlanAction:
    _static_frame_ids = {'map', 'odom'}

    def __init__(self, name):
        self.action_name = name
        self._as = actionlib.SimpleActionServer(self.action_name,
                ArmReachObjectAction,
                execute_cb=self.execute_cb,
                auto_start = False)
        self._feedback = ArmReachObjectFeedback()
        self._result = ArmReachObjectResult()
        self.trans = tf.TransformListener()
        self.chassis_move_client = actionlib.SimpleActionClient('simple_move', SimpleMoveAction)
        #self.chassis_move_client.wait_for_server()
        self.arm_move_client = actionlib.SimpleActionClient('arm_reach_position', ArmReachObjectAction)
        self.arm_move_client.wait_for_server()
        rospy.loginfo('Planner start')
        self._as.start()

    def execute_cb(self, goal):
        goal_pos_stamped = goal.pos
        goal_point = goal_pos_stamped
        #if goal_pos_stamped.header.frame_id in ArmPlanAction._static_frame_ids:
        #    goal_pos_stamped.header.stamp = rospy.Time(0)
        #while not rospy.is_shutdown():
        #    try:
        #        goal_point = self.trans.transfromPoint('arm_origin_link', 
        #            goal_pos_stamped)
        #    except Exception as e:
        #        rospy.logwarn(e.message)
        origin_goal = goal_point.point
        rospy.loginfo('target %f %f %f', goal_point.point.x, 
                goal_point.point.y, goal_point.point.z)
        # first move chassis over y
        if (fabs(goal_point.point.y) > 0.03):
            chassis_result = self.move_chassis(0, goal_point.point.y, 0)
            if not chassis_result.success:
                rospy.logwarn('failed to move chassis')
            goal_point.point.y -= chassis_result.moved_distance.y
        # now move arm
        rospy.loginfo('arm %f %f %f', goal_point.point.x, 
                goal_point.point.y, goal_point.point.z)
        arm_goal = goal
        arm_goal.pos = goal_point
        self.arm_move_client.send_goal(arm_goal)
        self.arm_move_client.wait_for_result()
        arm_result = self.arm_move_client.get_result()
        goal_point.point.x -= arm_result.moved.x
        goal_point.point.y -= arm_result.moved.y
        goal_point.point.z -= arm_result.moved.z
        rospy.loginfo('arm moved %f %f %f', arm_result.moved.x, 
                arm_result.moved.y, arm_result.moved.z)
        self._result.is_reached = arm_result.is_reached
        # finally move chassis over x
        if not arm_result.is_reached:
            chassis_result = self.move_chassis(goal_point.point.x, 0, 0)
            goal_point.point.x -= chassis_result.moved_distance.x
            self._result.is_reached = chassis_result.success
        self._result.moved.x = origin_goal.x - goal_point.point.x
        self._result.moved.y = origin_goal.y - goal_point.point.y
        self._result.moved.z = origin_goal.z - goal_point.point.z
        if self._result.is_reached:
            self._as.set_succeeded(self._result)
        else:
            self._as.set_aborted(self._result)
    
    def move_chassis(self, x, y, theta):
        goal = SimpleMoveGoal()
        goal.target.x = x
        goal.target.y = y
        goal.target.theta = theta
        self.chassis_move_client.send_goal(goal)
        return self.chassis_move_client.get_result()


def init():
    rospy.init_node('arm_planner')
    arm_planner = ArmPlanAction('arm_planner')
    rospy.spin()

if __name__ == '__main__':
    init()
