#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import sys
# Make sure that caffe is in the python path:
caffe_root = '/opt/ros/melodic/share/OpenCV-4.2.0-dev/'
sys.path.insert(0, caffe_root + 'python/')
import caffe

MODE = "COCO"
if MODE == "MPI":
    protoFile = "/opt/ros/melodic/share/OpenCV-4.2.0-dev/samples/dnn/pose_deploy_linevec_mpi.prototxt"
    weightsFile = "/opt/ros/melodic/share/OpenCV-4.2.0-dev/samples/dnn/pose_iter_160000.caffemodel"
    nPoints = 15
    POSE_PAIRS = [[0, 1], [1, 2], [2, 3], [3, 4], [1, 5], [5, 6], [6, 7], [1, 14], [14, 8], [8, 9], [9, 10], [14, 11], [11, 12], [12, 13]]
else:
    protoFile = "/opt/ros/melodic/share/OpenCV-4.2.0-dev/samples/dnn/pose_deploy_linevec_coco.prototxt"
    weightsFile = "/opt/ros/melodic/share/OpenCV-4.2.0-dev/samples/dnn/pose_iter_440000.caffemodel"
    nPoints = 18
    POSE_PAIRS = [[1, 2], [2, 3], [3, 4], [1, 5], [5, 6], [6, 7], [1, 8], [8, 9], [9, 10], [10, 11], [8, 12], [12, 13], [13, 14], [1, 0], [0, 15], [15, 17], [0, 16], [16, 18], [2, 17], [5, 18]]

# Load network
net = cv2.dnn.readNetFromCaffe(protoFile, weightsFile)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)

# Define subscriber node
class PoseDetectionNode:
    def __init__(self):
        self.pose_pub = rospy.Publisher('/poses', Image, queue_size=1)
        self.bridge = CvBridge()

    def callback(self):
        try:
            # Access laptop camera feed
            cap = cv2.VideoCapture(0)
            while True:
                # Read frame from camera
                ret, cv_image = cap.read()

                # Get image size
                (h, w) = cv_image.shape[:2]

                # Prepare input
                inputBlob = cv2.dnn.blobFromImage(cv2.resize(cv_image, (368,368)), 1.0/255, (368,368), (0,0,0), swapRB=False, crop=False)

                # Set input to network
                net.setInput(inputBlob)

                # Do forward pass
                output = net.forward()

                # Extract points
                points = []
                for i in range(nPoints):
                    # Get confidence map and extract local maxima
                    probMap = output[0, i, :, :]
                    probMap = cv2.resize(probMap, (w, h))

                    # Find global maxima
                    minVal, prob, minLoc, point = cv2.minMaxLoc(probMap)

                    # Add point if confidence is above threshold
                    if prob > 0.3:
                        points.append((int(point[0]), int(point[1])))
                    else:
                        points.append(None)

                # Draw skeleton
                for pair in POSE_PAIRS:
                    partA = pair[0]
                    partB = pair[1]

                    if points[partA] and points[partB]:
                        cv2.line(cv_image, points[partA], points[partB], (0, 255, 255), 2)
                        cv2.circle(cv_image, points[partA], 5, (0, 0, 255), thickness=-1, lineType=cv2.FILLED)

                # Publish image with skeleton
                self.pose_pub.publish(self.bridge.cv2_to_imgmsg(cv_image))

                # Quit window with 'q' key
                if cv2.waitKey(1) == ord('q'):
                    break

            # Release camera and close window
            cap.release()
            cv2.destroyAllWindows()

        except CvBridgeError as e:
            print(e)

# Main function
if __name__ == '__main__':
    rospy.init_node('pose_detection_node', anonymous=True)
    pose_detection_node = PoseDetectionNode()
    pose_detection_node.callback()

