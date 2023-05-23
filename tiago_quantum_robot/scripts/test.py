#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class DistanceTracker:
    def __init__(self):
        # Initialize the transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize the distance publisher
        self.distance_pub = rospy.Publisher('/distance', Float32, queue_size=1)

        # Initialize the pose subscriber
        self.pose_sub = rospy.Subscriber('/detected_aruco_pose', PoseStamped, self.pose_callback)

        # Initialize the time of the last received message
        self.last_time_received = rospy.get_time()

        # Initialize the timer for checking if a message has been received
        self.check_timer = rospy.Timer(rospy.Duration(1.0), self.check_received)

        # Set the maximum distance for normalizing the distance value
        self.max_distance = 2.0

    def pose_callback(self, pose_msg):
        try:
            # Look up the transform from the robot camera frame to the pose frame
            transform = self.tf_buffer.lookup_transform('xtion_rgb_frame', pose_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

            translation = transform.transform.translation

            # Calculate the distance between the frames
            distance = (translation.x ** 2 + translation.y ** 2 + translation.z ** 2) ** 0.5

            # Normalize the distance between 0 and 1
            normalized_distance = distance

            # Publish the normalized distance
            self.distance_pub.publish(Float32(normalized_distance))

            # Update the last time a message was received
            self.last_time_received = rospy.get_time()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to lookup transform for marker frame %s", pose_msg.header.frame_id)

    def publish_zero_distance(self):
        self.distance_pub.publish(Float32(0.0))

    def check_received(self, event):
        time_since_last = rospy.get_time() - self.last_time_received

        if time_since_last > 1.0: # Change the time threshold as needed
            self.publish_zero_distance()

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('distance_tracker')

    # Create an instance of the DistanceTracker class
    distance_tracker = DistanceTracker()

    # Start the node and run indefinitely
    rospy.spin()
