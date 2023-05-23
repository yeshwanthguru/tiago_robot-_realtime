#!/usr/bin/env python

import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from std_msgs.msg import String

class Robot:
    def __init__(self):
        self.play_m_as = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.play_m_as.wait_for_server()

    def prepare_robot(self, decision):
        rospy.loginfo("Stopping arm_controller...")
        rospy.sleep(1.0) # wait for arm_controller to stop
        rospy.loginfo("Unfold arm safely")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'reach_max_right'
        pmg.skip_planning = False
        self.play_m_as.send_goal(pmg)
        self.play_m_as.wait_for_result()
        rospy.loginfo("Grasp prepared.")
        return decision
    
    def pick_pose(self):
        rospy.loginfo("Picking final pose")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pick_final_pose'
        pmg.skip_planning = False
        self.play_m_as.send_goal(pmg)
        self.play_m_as.wait_for_result()

def callback(msg):
    decision = msg.data
    rospy.loginfo("Received decision: {}".format(decision))
    robot = Robot()
    robot.prepare_robot(decision)
    robot.pick_pose()
    pub = rospy.Publisher('/decision_process', String, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
         pub.publish("Decision achieved.")
         rate.sleep()

def listener():
    rospy.init_node('listener', anonymous=True)
    global robot ,grasp_pose_sub
    robot = Robot()
    grasp_pose_sub = rospy.Subscriber('/decision_made', String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException as e:
        rospy.logwarn("Exception occurred: {}".format(str(e)))
        rospy.signal_shutdown("Exception occurred")
