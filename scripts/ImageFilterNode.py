#!/usr/bin/env python3
# import cv2
# from cv_bridge.boost.cv_bridge_boost import getCvType

from numpy import frombuffer, reshape  # , ones, sign

import rospy
# from math import sin
# import timeit
from cv_bridge import CvBridgeError  # , CvBridge
from geometry_msgs.msg import TwistStamped  # , Twist
from scripts.Filters.ImageFilterThreshold import *
from scripts.ImagePositioningController import ImagePositioningController
from sensor_msgs.msg import Image
from std_msgs.msg import Bool  # ,Float64, String, UInt32MultiArray
from scripts.Filters.FilterHelper import display_process_timer


class ImageFilterNode:
    """
    A class for defining a ROS node to filter ultrasound images.
    """

    def __init__(self, filtering_rate=-1, debug_mode=False, analysis_mode=False):
        """
        Create a ROS node to filter raw ultrasound images and publish data about them.

        Parameters
        ----------
        filtering_rate: float
            rate at which the image filter will attempt to filter images. Defaults to as fast as possible.

        debug_mode: bool
            display graphics and additional print statements helpful in the debugging process.

        analysis_mode: bool
            display the time key processes take to occur.
        """

        # set parameters assisting in measuring and debugging code
        self.debug_mode = debug_mode
        self.analysis_mode = analysis_mode

        # set rate of message filtering
        # if the filtering rate is 0, filter images as fast as possible
        if filtering_rate == 0:
            filtering_rate = -1
        self.filtering_rate = filtering_rate  # hz

        # initialize ros node
        rospy.init_node('image_filtering')

        # Create a subscriber to receive the ultrasound images
        self.raw_image_subscriber = rospy.Subscriber('/Clarius/US', Image, self.raw_image_callback)

        # Create a subscriber to receive the command to start and stop filtering images
        self.filter_images_subscriber = rospy.Subscriber('/command/filter_images', Bool, self.filter_images_callback)

        # Create a publisher to publish the error of the centroid
        self.image_based_control_input_publisher = rospy.Publisher('/control_input/image_based', TwistStamped,
                                                                   queue_size=1)

        # Create a publisher to publish if the thyroid is visible in the image
        self.is_thyroid_in_image_status_publisher = rospy.Publisher('/status/thyroid_shown', Bool, queue_size=1)

        # Create a publisher to publish if the thyroid is centered in the image
        self.is_thyroid_centered_status_publisher = rospy.Publisher('/status/thyroid_centered', Bool, queue_size=1)

        # Create a publisher to publish the masked image
        # mask_publisher = rospy.Publisher('/image_data/mask', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the mask overlaid image
        # mask_overlay_publisher = rospy.Publisher('image_data/mask_overlay', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the thyroid contours
        # contour_publisher = rospy.Publisher('/image_data/contours', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the centroids of the thyroid
        # centroid_publisher = rospy.Publisher('/image_data/centroids', UInt32MultiArray, queue_size=1)

        # flag showing when a new image to filter is available
        self.is_new_image_available = False

        # store the data about the most recently sent image
        self.image_data = None

        # store the image filter used in the node
        self.image_filter = ImageFilterThreshold(debug_mode=False, analysis_mode=False)

        # store an image positioning controller object to calculate the image centroid error
        self.image_positioning_controller = ImagePositioningController(debug_mode=False,
                                                                       analysis_mode=False)

        # store the time when the last image sample was filtered
        self.time_of_last_image_filtered = time()

        # store current image filtering status
        self.filter_images = True

    # Filter the new image data, calculate the error of the centroid, and publish it
    def analyze_image(self):
        """
        Analyzes image saved as image_data object in node and publishes data about it.

        If the thyroid is not found in the image, the method publishes empty data.
        """

        # save the start of the process time
        start_of_process_time = time()

        # filter the image
        self.image_data = self.image_filter.fully_filter_image(self.image_data)

        # note the time required to fully filter the image
        start_of_process_time = display_process_timer(start_of_process_time, "Time to fully filter", self.analysis_mode)

        # Display the image mask generated from filtering process
        # cv2.imshow("generated mask", self.image_data.image_mask)
        # cv2.waitKey(1)

        # Determine if the thyroid is present in the image
        thyroid_in_image = (len(self.image_data.contours_in_image) > 0 and
                            len(self.image_data.contours_in_image) == len(self.image_data.contour_centroids))

        # Publish if the thyroid is in the image
        self.is_thyroid_in_image_status_publisher.publish(Bool(thyroid_in_image))

        # note the time required to determine and publish the status of the thyroid in the image
        start_of_process_time = display_process_timer(start_of_process_time, "Thyroid status check time",
                                                      self.analysis_mode)

        # If the thyroid is present in the image, publish data about it
        if thyroid_in_image:

            # Analyze the centroid location to determine the needed control input, current thyroid position error, and
            # if the thyroid is in the center of the image
            control_input_msg, current_error, is_thyroid_centered = self.image_positioning_controller. \
                calculate_control_input(self.image_data)

            # note the time required to calculate the image based control input
            start_of_process_time = display_process_timer(start_of_process_time,
                                                          "Image based control input calculation",
                                                          self.analysis_mode)

            if self.debug_mode:
                # Create a temporary recolored image for debugging purposes
                temp_masked_result = cv2.cvtColor(self.image_data.original_image, cv2.COLOR_GRAY2RGB)

                # Add mask to original image
                temp_masked_result = cv2.drawContours(temp_masked_result, self.image_data.contours_in_image[0],
                                                      -1, (0, 255, 0), -1)

                # Add centroid to masked image
                temp_masked_result = cv2.circle(temp_masked_result, self.image_data.contour_centroids[0], 6, (255, 0, 0),
                                                -1)

                # Add lines showing where centroid should be
                temp_masked_result = cv2.line(temp_masked_result, (0, 240), (640, 240), (0, 0, 255), 2)
                temp_masked_result = cv2.line(temp_masked_result, (320, 0), (320, 480), (0, 165, 255), 2)

                # note the time required to mark up the original image for display
                start_of_process_time = display_process_timer(start_of_process_time, "Markup Time", self.analysis_mode)

                # Show masked image
                cv2.imshow("Marked-up image", temp_masked_result)
                cv2.waitKey(1)

                # note the time required to update the visualization
                display_process_timer(start_of_process_time, "Display Time", self.analysis_mode)

            # Publish if the thyroid is centered
            self.is_thyroid_centered_status_publisher.publish(Bool(is_thyroid_centered))

            # publish the image based control input
            self.image_based_control_input_publisher.publish(control_input_msg)

        # Publish no control input and do not show the thyroid as centered
        else:
            self.is_thyroid_centered_status_publisher.publish(Bool(False))
            self.image_based_control_input_publisher.publish(TwistStamped())

        # set that a new image has not been delivered yet
        self.is_new_image_available = False

    # Callback function for evaluating new image data
    def raw_image_callback(self, data: Image):
        """
        Regulates filtering rate and updates object parameters.
        """

        # get the current time
        current_time = time()

        # check if the image needs to be filtered
        time_since_last_image = current_time - self.time_of_last_image_filtered

        if time_since_last_image > (1 / self.filtering_rate) and self.filter_images:

            # record the current time
            self.time_of_last_image_filtered = current_time

            # record the current time for timing of processes
            start_of_process_time = current_time

            try:
                """# Convert image using CvBridge
                # convert the image message to a cv2 data array
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')"""

                # Convert image using numpy tools
                cv_image = frombuffer(data.data, dtype=uint8)
                cv_image = reshape(cv_image, (data.height, data.width))

                # note the amount of time required to convert the image
                start_of_process_time = display_process_timer(start_of_process_time, "Image conversion time",
                                                              self.analysis_mode)

                # Show original image received from the ROS topic
                # cv2.imshow("original image", cv_image)
                # cv2.waitKey(1)

            except CvBridgeError:
                print("Image error.")
                self.image_data = None
                return

            # Create new image data based on received image
            self.image_data = ImageData(image_data=cv_image)

            # note the amount of time required to create an image data object
            start_of_process_time = display_process_timer(start_of_process_time, "Image_data object creation time",
                                                          self.analysis_mode)

            # mark that a new image is available
            self.is_new_image_available = True

            # analyze the new image
            self.analyze_image()

            # note the amount of time required to analyze the image
            display_process_timer(start_of_process_time, "Image analysis time", self.analysis_mode)

    # Define callback for updating filter images command
    def filter_images_callback(self, data: Bool):
        """
        Update filter_images_command parameter.
        """
        self.filter_images = data.data


if __name__ == '__main__':
    # create node object
    filter_node = ImageFilterNode(filtering_rate=10, debug_mode=True, analysis_mode=True)

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # spin until node is terminated
    # callback function handles image analysis
    rospy.spin()


        
