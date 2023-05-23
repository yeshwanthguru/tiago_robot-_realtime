#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates,ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import Twist




def spawn_random_aruco_cube():
    # Initialize ROS node
    rospy.init_node('spawn_random_aruco_cube')

    # Get the current position and orientation of the aruco_cube object
    aruco_cube_pose = rospy.wait_for_message('/gazebo/model_states', ModelStates)
    aruco_cube_index = aruco_cube_pose.name.index('aruco_cube')
    aruco_cube_position = aruco_cube_pose.pose[aruco_cube_index].position
    aruco_cube_orientation = aruco_cube_pose.pose[aruco_cube_index].orientation

    # Generate random offsets for x and y coordinates
    x_offset = random.uniform(-0.05, 0.05)
    y_offset = random.uniform(-0.05, 0.05)

    # Calculate new position
    new_x = aruco_cube_position.x + x_offset
    new_y = aruco_cube_position.y + y_offset
    new_z = aruco_cube_position.z
    new_position = Pose()
    new_position.position.x = new_x
    new_position.position.y = new_y
    new_position.position.z = new_z
    new_position.orientation = aruco_cube_orientation

    # Set new position for aruco_cube
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    aruco_cube_state = ModelState()
    aruco_cube_state.model_name = 'aruco_cube'
    aruco_cube_state.pose = new_position
    aruco_cube_state.twist = Twist()
    aruco_cube_state.reference_frame = 'world'
    set_model_state(aruco_cube_state)

if __name__ == '__main__':
    try:
        spawn_random_aruco_cube()
    except rospy.ROSInterruptException:
        pass

