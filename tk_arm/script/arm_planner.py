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

    def init(self, name):
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
        self.arm_move_client = actionlib.SimpleActionClient('arm_reach_position', ArmReachObjectAction)
        self.arm_move_client.wait_for_server()
        self._as.start()

    def execute_cb(self, goal):
        goal_pos_stamped = goal.pos
        goal_point = goal_pos_stamped
        if goal_pos_stamped.header.frame_id in ArmPlanAction._static_frame_ids:
            goal_pos_stamped.header.stamp = rospy.Time(0)
        while not rospy.is_shutdown():
            try:
                goal_point = self.trans.transfromPoint('arm_origin_link', 
                    goal_pos_stamped)
            except Exception as e:
                rospy.logwarn(e.message)
        rospy.loginfo('target %f %f %f', goal_point.point.x, 
                goal_point.y, goal_point.point.z)
        if (fabs(goal_point.point.y) > 0.03):
            if not self.move_y(goal_point.point.y):
                rospy.logwarn('failed to move chassis, aborted')
                self._result.is_reached = False
                self._as.set_aborted(self._result)
                return
            goal_point.point.y = 0
        arm_goal = goal
        arm_goal.pos = goal_point
        self.arm_move_client.send_goal(arm_goal)
        if self.arm_move_client.wait_for_result():
            rospy.logwarn('arm failed to reach, aborted')
            self._result.is_reached = False
            self._as.set_aborted(self._result)
        else:
            self._result.is_reached = True
            self._as.set_succeeded(self._result)
    
    def move_y(self, y):
        goal = SimpleMoveGoal()
        goal.target.x = 0
        goal.target.y = y
        goal.target.theta = 0
        self.chassis_move_client.send_goal(goal)
        return self.chassis_move_client.wait_for_result()

def init():
    rospy.init_node('arm_planner')
    arm_planner = ArmPlanAction('arm_planner')
    rospy.spin()

if __name__ == '__main__':
    init()
