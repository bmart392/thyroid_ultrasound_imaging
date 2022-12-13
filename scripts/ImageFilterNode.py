#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String, UInt32MultiArray
from sensor_msgs.msg import Image
from numpy import sign
from ImageFilter import *
from ImagePositioningController import ImagePositioningController
from math import sin
import timeit
from cv_bridge import CvBridge


def raw_image_callback(data, node):
    node.image_data = ImageData(CvBridge().imgmsg_to_cv2(data))
    node.is_new_image_available = True


class ImageFilterNode:

    def __init__(self):

        # flag showing when a new image to filter is available
        self.is_new_image_available = False

        # store the data about the most recently sent image
        self.image_data = None

        # store the image filter used in the node
        self.image_filter = ImageFilter()

        # store an image positioning controller object to calculate the image centroid error
        self.image_positioning_controller = ImagePositioningController()

        # initialize ros node
        rospy.init_node('image_filtering')

        # Create a subscriber to receive the ultrasound images
        self.raw_image_subscriber = rospy.Subscriber('/Clarius/US', Image, raw_image_callback, self)

        # Create a publisher to publish the error of the centroid
        self.image_centroid_error_publisher = rospy.Publisher('/image_data/centroid_error', Float64, queue_size=1)

        # Create a publisher to publish the masked image
        # mask_publisher = rospy.Publisher('/image_data/mask', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the mask overlaid image
        # mask_overlay_publisher = rospy.Publisher('image_data/mask_overlay', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the thryoid contours
        # contour_publisher = rospy.Publisher('/image_data/contours', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the centroids of the thyroid
        # centroid_publisher = rospy.Publisher('/image_data/centroids', UInt32MultiArray, queue_size=1)

    # Filter the new image data, calculate the error of the centroid, and publish it
    def analyze_image(self):

        # filter the image
        self.image_filter.fully_filter_image(self.image_data)

        # calculate the error of the centroids in meters
        x_error = Float64(self.image_positioning_controller.calculate_error_in_meters(self.image_data))

        # publish the centroid error
        self.image_centroid_error_publisher.publish(x_error)


if __name__ == '__main__':

    # create node object
    filter_node = ImageFilterNode()

    print("Node initialized.")

    # Initialize rate of image filtering
    rate = rospy.Rate(2)

    # While not shutdown
    while not rospy.is_shutdown():

        print("Waiting for new data.")

        # If a new image is available
        if filter_node.is_new_image_available:

            # Filter and publish it
            filter_node.analyze_image()

            # Note that the existing image data has already been filtered
            filter_node.is_new_image_available = False

        # Wait to publish the next message
        rate.sleep()


        
