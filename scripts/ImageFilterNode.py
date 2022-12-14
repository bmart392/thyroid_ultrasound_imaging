#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64, String, UInt32MultiArray, Bool
from sensor_msgs.msg import Image
from numpy import sign
from ImageFilter import *
from ImagePositioningController import ImagePositioningController
from math import sin
import timeit
from cv_bridge import CvBridge


class ImageFilterNode:

    def __init__(self):

        # initialize ros node
        rospy.init_node('image_filtering')

        # Create a subscriber to receive the ultrasound images
        self.raw_image_subscriber = rospy.Subscriber('/Clarius/US', Image, self.raw_image_callback)

        # Create a publisher to publish the error of the centroid
        self.image_based_control_input_publisher = rospy.Publisher('/control_input/image_based', TwistStamped, queue_size=1)

        # Create a publisher to publish if the thyroid is visible in the image
        self.is_thyroid_in_image_status_publisher = rospy.Publisher('/status/thyroid_shown', Bool, queue_size=1)

        # Create a publisher to publish if the thyroid is centered in the image
        self.is_thyroid_centered_status_publisher = rospy.Publisher('/status/thyroid_centered', Bool, queue_size=1)

        # Create a publisher to publish the masked image
        # mask_publisher = rospy.Publisher('/image_data/mask', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the mask overlaid image
        # mask_overlay_publisher = rospy.Publisher('image_data/mask_overlay', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the thryoid contours
        # contour_publisher = rospy.Publisher('/image_data/contours', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the centroids of the thyroid
        # centroid_publisher = rospy.Publisher('/image_data/centroids', UInt32MultiArray, queue_size=1)

        # flag showing when a new image to filter is available
        self.is_new_image_available = False

        # store the data about the most recently sent image
        self.image_data = None

        # store the image filter used in the node
        self.image_filter = ImageFilter()

        # store an image positioning controller object to calculate the image centroid error
        self.image_positioning_controller = ImagePositioningController()

    # Filter the new image data, calculate the error of the centroid, and publish it
    def analyze_image(self):

        # filter the image
        self.image_data = self.image_filter.fully_filter_image(self.image_data)

        # Publish if the thyroid is in the image
        self.is_thyroid_in_image_status_publisher.publish(
            Bool(
                len(self.image_data.contour_centroids) > 0))

        # Analyze the centroid location to determine the needed control input, current thyroid position error, and
        # if the thyroid is in the center of the image
        control_input_msg, current_error, is_thyroid_centered = self.image_positioning_controller.\
            calculate_control_input(self.image_data)

        # Publish if the thyroid is centered
        self.is_thyroid_centered_status_publisher.publish(Bool(is_thyroid_centered))

        # publish the image based control input
        self.image_based_control_input_publisher.publish(control_input_msg)

        # set that a new image has not been delivered yet
        self.is_new_image_available = False

    # Callback function for evaluating new image data
    def raw_image_callback(self, data):

        # convert the image message to a cv2 data array
        self.image_data = ImageData(CvBridge().imgmsg_to_cv2(data))

        # mark that a new image is available
        self.is_new_image_available = True

        # analyze the new image
        self.analyze_image()


if __name__ == '__main__':

    # create node object
    filter_node = ImageFilterNode()

    print("Node initialized.")

    # spin until node is terminated
    # callback function handles image analysis
    rospy.spin()


        
