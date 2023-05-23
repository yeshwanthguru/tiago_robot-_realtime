#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose
import tf2_ros

rospy.init_node('aruco_detection_publisher')

class ArucoDetector:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
        self.detected_pose_pub = rospy.Publisher('/detected_aruco_pose', PoseStamped, queue_size=1)
        self.detections_count = 0
        self.log_file = open('/home/guru/tiago_public_ws/src/tiago_quantum_robot/data/aruco_pose.log', 'w+')


    def strip_leading_slash(self, s):
        return s[1:] if s.startswith("/") else s

    def detect_aruco(self):
        self.detections_count += 1
        rospy.loginfo("Waiting for an Aruco detection...")
        aruco_pose = rospy.wait_for_message('/aruco_single/pose', PoseStamped)
        rospy.loginfo("Got: " + str(aruco_pose))
        aruco_pose.header.frame_id = self.strip_leading_slash(aruco_pose.header.frame_id)

        rospy.loginfo("Transforming from frame: " + aruco_pose.header.frame_id + " to 'base_footprint'")
        ps = PoseStamped()
        ps.pose.position = aruco_pose.pose.position
        ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)
        ps.header.frame_id = aruco_pose.header.frame_id

        transform_ok = False
        while not transform_ok and not rospy.is_shutdown():
            try:
                transform = self.tfBuffer.lookup_transform("base_footprint", ps.header.frame_id, rospy.Time(0))
                aruco_ps = do_transform_pose(ps, transform)
                transform_ok = True
            except tf2_ros.ExtrapolationException as e:
                rospy.logwarn("Exception on transforming point... trying again \n(" + str(e) + ")")
                rospy.sleep(0.01)
                ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)

        self.detected_pose_pub.publish(aruco_ps)

        # Log to file
        self.log_file.write(str(aruco_ps) + '\n')
        self.log_file.flush()

        # Print to the terminal every 10 detections
        if self.detections_count % 10 == 0:
            rospy.loginfo("Aruco pose: " + str(aruco_ps))

if __name__ == '__main__':
    aruco_detector = ArucoDetector()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        aruco_detector.detect_aruco()
        rate.sleep()
