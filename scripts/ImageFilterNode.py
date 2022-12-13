#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String, UInt32MultiArray
from numpy import sign
from ImageFilter import *
from math import sin
import timeit


def callback(data):
    global image_data
    image_data = ImageData(image_data=data)


if __name__ == '__main__':

    # initialize ros node
    rospy.init_node('image_filtering')

    # Create a subscriber to receive the ultrasound images
    raw_image_subscriber = rospy.Subscriber('/image_data/raw', UInt32MultiArray, callback)

    # Create a publisher to publish the error of the centroid
    image_centroid_error_publisher = rospy.Publisher('/image_data/centroid_error', Float64, queue_size=1)
    
    # Create a publisher to publish the masked image
    # mask_publisher = rospy.Publisher('/image_data/mask', UInt32MultiArray, queue_size=1)

    # Create a publisher to publish the mask overlaid image
    # mask_overlay_publisher = rospy.Publisher('image_data/mask_overlay', UInt32MultiArray, queue_size=1)
    
    # Create a publisher to publish the thryoid contours
    # contour_publisher = rospy.Publisher('/image_data/contours', UInt32MultiArray, queue_size=1)

    # Create a publisher to publish the centroids of the thyroid
    # centroid_publisher = rospy.Publisher('/image_data/centroids', UInt32MultiArray, queue_size=1)

    
    # Create an image filter to use for segmenting the images
    image_filter = ImageFilter()

    # Initialize image data variable
    image_data = None

    # Initialize rate of image filtering
    rate = rospy.Rate(2)

    # Record the time the node started publishing
    start_time = timeit.default_timer()

    while not rospy.is_shutdown():

        
        # Generate a centroid error message
        centroid_error = Float64()
        
        # Assign the error to the message
        centroid_error.data = sin(timeit.default_timer() - start_time)

        # Publish the error
        image_centroid_error_publisher.publish(centroid_error)

        # Wait to publish the next message
        rate.sleep()

        # # If there is an image to filter
        # if image_data is not None:

        #     # Fully filter the image to generate all versions of the image and
        #     image_filter.fully_filter_image(image_data)

        #     # Calculate contours and centroids of image
        #     image_data.generate_contours_in_image()
        #     image_data.calculate_image_centroids()

        #     # Generate mask overlaid image
        #     image_data.generate_mask_overlaid_image()

        # Publish contours
        # Publish centroids
        # Publish masked image
        # Publish centroid overlaid image
        # Publish size of image where centroids are found


        
