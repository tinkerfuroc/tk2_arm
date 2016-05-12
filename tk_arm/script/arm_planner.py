#!/usr/bin/python

import actionlib
import rospy
from tk_arm.msg._ArmPathGoal import ArmPathGoal
from tk_arm.msg._ArmPathFeedback import ArmPathFeedback
from tk_arm.msg._ArmPathResult import ArmPathResult
from tk_arm.msg._ArmPathAction import ArmPathAction
# from tk_arm.msg._SimpleMoveGoal import SimpleMoveGoal
# from tk_arm.msg._SimpleMoveAction import SimpleMoveAction
from tinker_msgs.msg._SimpleMoveGoal import SimpleMoveGoal
from tinker_msgs.msg._SimpleMoveAction import SimpleMoveAction
import tf
from math import fabs

class ArmPlanAction:
    _static_frame_ids = {'map', 'odom'}

    def __init__(self, name):
        self.action_name = name
        self._as = actionlib.SimpleActionServer(self.action_name,
                ArmPathAction,
                execute_cb=self.execute_cb,
                auto_start = False)
        self._feedback = ArmPathFeedback()
        self._result = ArmPathResult()
        self.trans = tf.TransformListener()
        self.chassis_move_client = actionlib.SimpleActionClient('simple_move', SimpleMoveAction)
        #self.chassis_move_client.wait_for_server()
        self.arm_move_client = actionlib.SimpleActionClient('arm_path', ArmPathAction)
        self.arm_move_client.wait_for_server()
        rospy.loginfo('Planner start')
        self._as.start()

    def execute_cb(self, goal):
        goal_point = goal.path.poses[-1]
        # rospy.loginfo('target raw %f %f %f', goal_point.point.x, 
        #         goal_point.point.y, goal_point.point.z)
        if goal_point.header.frame_id in ArmPlanAction._static_frame_ids:
            goal_point.header.stamp = rospy.Time(0)
        while not rospy.is_shutdown():
            try:
                goal_point = self.trans.transformPoint('arm_origin_link', 
                    goal_point)
                break
            except Exception as e:
                rospy.logwarn(e.message)
        # origin_goal = goal.path
        # rospy.loginfo('target %f %f %f', goal_point.point.x, 
        #         goal_point.point.y, goal_point.point.z)
        # # first move chassis over y
        # if (fabs(goal_point.point.y) > 0.03):
        #     chassis_result = self.move_chassis(0, goal_point.point.y, 0)
        #     if not chassis_result.success:
        #         rospy.logwarn('failed to move chassis')
        #     goal_point.point.y -= chassis_result.moved_distance.y
        # Move arm first
        rospy.loginfo('arm goal %f %f %f', goal_point.point.x, 
                goal_point.point.y, goal_point.point.z)
        arm_goal = goal
        arm_goal.path = goal.path
        # arm_goal.grasp_state &= 0x01 # do not force grasp in case of failure
        self.arm_move_client.send_goal(arm_goal)
        self.arm_move_client.wait_for_result()
        arm_result = self.arm_move_client.get_result()
        rospy.loginfo('arm moved %4.2f %4.2f %4.2f', arm_result.moved.x, 
                arm_result.moved.y, arm_result.moved.z)
        goal_point.point.x -= arm_result.moved.x
        goal_point.point.y -= arm_result.moved.y
        goal_point.point.z -= arm_result.moved.z
        self._result.is_reached = arm_result.is_reached
        # finally move chassis over x and y
        if not arm_result.is_reached:
            rospy.loginfo('chassis move [x,y] = [%4.2f,%4.2f]', 
                    goal_point.point.x, goal_point.point.y)
            chassis_result = self.move_chassis(goal_point.point.x, goal_point.point.y, 0)
            goal_point.point.x -= chassis_result.moved_distance.x
            goal_point.point.y -= chassis_result.moved_distance.y
            self._result.is_reached = chassis_result.success
        #     arm_goal = goal
        #     arm_goal.pos.point = arm_result.moved
        #     self.arm_move_client.send_goal(arm_goal)
        #     self.arm_move_client.wait_for_result()
        self._result.moved = arm_result.moved
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
        self.chassis_move_client.wait_for_result()
        return self.chassis_move_client.get_result()


def init():
    rospy.init_node('arm_planner')
    arm_planner = ArmPlanAction('arm_planner')
    rospy.spin()

if __name__ == '__main__':
    init()
