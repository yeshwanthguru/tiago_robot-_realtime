import cv2
import numpy as np
import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point

# Load the cascade classifier
cascade = cv2.CascadeClassifier('/home/guru/tiago_public_ws/src/tiago_quantum_robot/data/haarcascade_upperbody.xml')

# Create a video capture object for the camera
cap = cv2.VideoCapture(0)

# Initialize TF2 broadcaster and listener
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
tf_broadcaster = tf2_ros.TransformBroadcaster()

while True:
    # Read the frame from the camera
    ret, frame = cap.read()

    if not ret:
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect bodies using the cascade classifier
    bodies = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # Iterate over the detected bodies
    for (x, y, w, h) in bodies:
        # Calculate the centroid of the upper body
        body_centroid = (x + w // 2, y + h // 2)

        # Create a PointStamped message with the body centroid in the camera frame
        body_point = PointStamped()
        body_point.header.frame_id = 'camera_frame'
        body_point.header.stamp = rospy.Time.now()
        body_point.point = Point(body_centroid[0], body_centroid[1], 0.0)

        try:
            # Transform the body point to the world frame
            transformed_point = tf_buffer.transform(body_point, 'world_frame', rospy.Duration(1.0))

            # Extract the transformed centroid
            transformed_centroid = (transformed_point.point.x, transformed_point.point.y)

            # Calculate the distance between the body centroid and the camera
            distance = np.linalg.norm(np.array(body_centroid) - np.array(transformed_centroid))
            rospy.loginfo(distance)
            # Draw the rectangle and distance on the frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "Distance: {:.2f} units".format(distance), (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

    # Display the frame with detected bodies
    cv2.imshow('Body Detection', frame)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Release the video capture object and close the windows
cap.release()
cv2.destroyAllWindows()
