#!/usr/bin/env python

import rospy
import os
from geometry_msgs.msg import PoseStamped
import tf.transformations as tr

def transform_pose(pose_in):
    # Get the translation and rotation from the input pose
    pos_in = pose_in.pose.position
    ori_in = pose_in.pose.orientation
    # Transform the rotation by 180 degrees around the x-axis (pitch)
    mat_rot_x = tr.rotation_matrix(3.142, [1,0,0])
    rot_in = tr.quaternion_matrix([ori_in.x, ori_in.y, ori_in.z, ori_in.w])
    rot_out = mat_rot_x.dot(rot_in)
    ori_out = tr.quaternion_from_matrix(rot_out)
    # Construct the transformed pose
    pose_out = PoseStamped()
    pose_out.header = pose_in.header
    pose_out.pose.position = pos_in
    pose_out.pose.orientation.x = ori_out[0]
    pose_out.pose.orientation.y = ori_out[1]
    pose_out.pose.orientation.z = ori_out[2]
    pose_out.pose.orientation.w = ori_out[3]
    return pose_out

def callback(data):
    # Transform the received pose and publish the result
    transformed_pose = transform_pose(data)
    pub.publish(transformed_pose)   
if __name__ == '__main__':
    rospy.init_node('pose_transformer', anonymous=True)
    # Setup subscriber to detected_aruco_pose and publisher to transformed_pose
    detected_pose_sub = rospy.Subscriber('/detected_aruco_pose', PoseStamped, callback)
    pub = rospy.Publisher('/grasp_pose', PoseStamped, queue_size=10)
    rospy.spin()