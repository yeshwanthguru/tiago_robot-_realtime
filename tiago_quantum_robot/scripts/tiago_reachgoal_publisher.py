#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('publish_reach_goal')

human_present = False
publish_goal = False

cap = cv2.VideoCapture(0)
bridge = CvBridge()

reach_goal = JointTrajectory()
reach_goal.joint_names = ['torso_lift_joint','arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint']

point = JointTrajectoryPoint()
point.positions = [0.34,1.34, -0.46, 1.43, 0.30, 0.91, 0.08, 0.00]
point.time_from_start = rospy.Duration(10.0)

reach_goal.points.append(point)

pub = rospy.Publisher('/reach_goal', JointTrajectory, queue_size=10)
rate = rospy.Rate(10) # 10 Hz

while not rospy.is_shutdown():
    if not human_present: # detect human only once
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (21, 21), 0)
        _, thresh = cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY)

    cnts, hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    for c in cnts:
        if cv2.contourArea(c) > 500:
            # if area of contour is > 500, detected as human presence
            human_present = True
            rospy.loginfo("Published reach goal message")

            rate.sleep()
            break

    cv2.imshow('frame',frame) # display video stream

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    if human_present: # publish topic continuously
        pub.publish(reach_goal)

cap.release()
cv2.destroyAllWindows()

