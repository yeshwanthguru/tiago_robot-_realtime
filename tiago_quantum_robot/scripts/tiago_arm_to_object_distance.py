#!/usr/bin/env python
import rospy
from geometry_msgs.msg import  PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from std_msgs.msg import Float32
import math

def calculate_endeffector_distance():
    rospy.init_node('Arm_to_object_distance_calculator')
    buffer = Buffer()
    listener = TransformListener(buffer)

    def callback(data):
        try:
            # Transform the gripper_grasping_frame to the robot's base footprint frame
            gripper_pose = PoseStamped()            
            gripper_pose.header.frame_id = "gripper_grasping_frame"
            gripper_pose.pose.position.x = 0
            gripper_pose.pose.position.y = 0
            gripper_pose.pose.position.z = 0
            gripper_pose.pose.orientation.x = 0
            gripper_pose.pose.orientation.y = 0
            gripper_pose.pose.orientation.z = 0
            gripper_pose.pose.orientation.w = 1
            gripper_pose_in_base = tf2_geometry_msgs.do_transform_pose(gripper_pose,
                                                                      buffer.lookup_transform("base_footprint",
                                                                                              "gripper_grasping_frame",
                                                                                              rospy.Time(0),                                                                                              
                                                                                              rospy.Duration(1.0)))           
            #Transform the received pose to the TF2 frame
            aruco_pose_in_base = tf2_geometry_msgs.do_transform_pose(data,
                                                                     buffer.lookup_transform("base_footprint",
                                                                                             data.header.frame_id,
                                                                                             rospy.Time(0),
                                                                                             rospy.Duration(1.0)))

            # Calculate the distance between the gripper_grasping_frame and received pose
            distance = ((aruco_pose_in_base.pose.position.x - gripper_pose_in_base.pose.position.x) ** 2 
                        + (aruco_pose_in_base.pose.position.y - gripper_pose_in_base.pose.position.y) ** 2 
                        + (aruco_pose_in_base.pose.position.z - gripper_pose_in_base.pose.position.z) ** 2) ** 0.5

            # setting the minimum and maximum distance values as per the requirement
            min_distance = 0.00
            max_distance = 0.66 

            # map the distance to a value between -1 and 1 with a center point of 0
            mapped_value = ((distance - min_distance) / (max_distance - min_distance)) * 2 - 1
            mapped_value = abs(mapped_value)
            mapped_value = min(1, mapped_value)

            # publishing the mapped value on the '/mapped_distance' topic
            mapped_distance_pub = rospy.Publisher('/mapped_distance_tiago_arm_object', Float32, queue_size=10)
            mapped_distance_pub.publish(mapped_value)

            # publishing the original distance on the '/distance' topic
            distance_pub.publish(Float32(distance))

        except Exception as ex:
            rospy.logerr(ex)

    # Subscribe to the '/detected_aruco_pose' topic
    rospy.Subscriber('/grasp_pose', PoseStamped, callback)

    # Publish the original distance on the '/distance' topic
    distance_pub = rospy.Publisher('/distance_tiago_arm_object', Float32, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    calculate_endeffector_distance()
