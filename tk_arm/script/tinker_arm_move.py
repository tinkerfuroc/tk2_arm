#!/usr/bin/python

import actionlib
import rospy
from tk_arm.msg import ArmReachObjectGoal
from tk_arm.msg import ArmReachObjectFeedback
from tk_arm.msg import ArmReachObjectResult
from tk_arm.msg import ArmReachObjectAction
from tinker_msgs.msg import SimpleMoveGoal
from tinker_msgs.msg import SimpleMoveAction
from tk_arm.msg import ArmPathAction
from actionlib_msgs.msg import GoalStatus
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
        self.chassis_move_client.wait_for_server()
        rospy.loginfo('Action client \'simple_move\' started.')
        self.arm_move_client = actionlib.SimpleActionClient('arm_point', ArmReachObjectAction)
        self.arm_move_client.wait_for_server()
        rospy.loginfo('Action client \'arm_point\' started.')
        self.cancelled = False
        self._as.start()
        rospy.loginfo('Action server \'tinker_arm_move\' started.')

    def execute_cb(self, goal):
        self.cancelled = False
        goal_pos_stamped = goal.pos
        goal_point = goal_pos_stamped
        rospy.loginfo('target raw %f %f %f', goal_point.point.x, 
                goal_point.point.y, goal_point.point.z)
        if goal_point.header.frame_id in ArmPlanAction._static_frame_ids:
            goal_point.header.stamp = rospy.Time(0)
        while not rospy.is_shutdown():
            try:
                goal_point = self.trans.transformPoint('arm_origin_link', 
                    goal_point)
                break
            except Exception as e:
                rospy.logwarn(e.message)
        origin_goal = goal_point.point
        rospy.loginfo('target %f %f %f', origin_goal.x, 
                origin_goal.y, origin_goal.z)
        self._result.is_reached = True
        # Move arm first
        rospy.loginfo('arm goal %f %f %f', goal_point.point.x, 
                goal_point.point.y, goal_point.point.z)
        arm_goal = goal
        arm_goal.pos = goal_point 
        self.arm_move_client.send_goal(arm_goal)
        self.wait_or_cancel(self.arm_move_client)
        if self.cancelled:
            self._as.set_preempted()
            return
        arm_result = self.arm_move_client.get_result()
        rospy.loginfo('arm moved %4.2f %4.2f %4.2f', arm_result.moved.x, 
                arm_result.moved.y, arm_result.moved.z)
        goal_point.point.x -= arm_result.moved.x
        goal_point.point.y -= arm_result.moved.y
        goal_point.point.z -= arm_result.moved.z
        rospy.loginfo('remain %4.2f %4.2f %4.2f', goal_point.point.x,
                goal_point.point.y, goal_point.point.z)
        # finally move chassis over x and y
        if not arm_result.is_reached and not self.cancelled:
            rospy.loginfo('chassis move [x,y] = [%4.2f,%4.2f]', 
                    goal_point.point.x, goal_point.point.y)
            chassis_result = self.move_chassis(goal_point.point.x, goal_point.point.y, 0)
            if self.cancelled:
                self._as.set_preempted()
                return
            goal_point.point.x -= chassis_result.moved_distance.x
            goal_point.point.y -= chassis_result.moved_distance.y
            if not chassis_result.success:
                self._result.is_reached = False
        if fabs(goal_point.point.z) > 0.07:
            self._result.is_reached = False
        self._result.moved = arm_result.moved
        if self.cancelled:
            self._as.set_preempted()
        elif self._result.is_reached:
            self._as.set_succeeded(self._result)
        else:
            self._as.set_aborted(self._result)
    
    def move_chassis(self, x, y, theta):
        goal = SimpleMoveGoal()
        goal.target.x = x
        goal.target.y = y
        goal.target.theta = theta
        self.chassis_move_client.send_goal(goal)
        self.wait_or_cancel(self.chassis_move_client)
        return self.chassis_move_client.get_result()

    def wait_or_cancel(self, action_client):
        ### returns true if finished or aborted, false if server cancelled
        rate = rospy.Rate(20)
        while True:
            state = action_client.get_state()
            if state == GoalStatus.ABORTED or state == GoalStatus.SUCCEEDED:
                return True
            if self._as.is_preempt_requested():
                rospy.loginfo('[Tinker Arm Planner]Cancelling goal!!!')
                action_client.cancel_all_goals()
                self.cancelled = True
                return False
            rate.sleep()


def init():
    rospy.init_node('tinker_arm_move')
    arm_planner = ArmPlanAction('tinker_arm_move')
    rospy.spin()

if __name__ == '__main__':
    init()
