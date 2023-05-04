#!/usr/bin/env python3

"""
File containing ImageFiterNode class definition and ROS running code.
"""

# TODO Implement threading on all portions of code using imshow
# See: https://nrsyed.com/2018/07/05/multithreading-with-opencv-python-to-improve-video-processing-performance/

# Import standard packages
from numpy import frombuffer, reshape, uint8  # , ones, sign
from cv_bridge import CvBridgeError  # , CvBridge
from time import time
from copy import copy

# Import ROS packages
# import rospy
from rospy import init_node, spin, Subscriber, Publisher
from geometry_msgs.msg import TwistStamped  # , Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String  # ,Float64, String, UInt32MultiArray

# Import custom objects
from thyroid_ultrasound_imaging.ImageData.ImageData import ImageData

from thyroid_ultrasound_imaging.ImageFilter.ImageFilterThreshold import ImageFilterThreshold
from thyroid_ultrasound_imaging.ImageFilter.ImageFilterGrabCut import ImageFilterGrabCut
from thyroid_ultrasound_imaging.ImageFilter.FilterConstants import COLOR_GRAY

from thyroid_ultrasound_imaging.Visualization.VisualizationConstants import *
from thyroid_ultrasound_imaging.Visualization.Visualization import Visualization

from thyroid_ultrasound_imaging.Controller.ImagePositioningController import ImagePositioningController
from thyroid_ultrasound_imaging.Visualization.display_process_timer import display_process_timer

# Define the types of image filters that could be used
THRESHOLD_FILTER: int = 0
GRABCUT_FILTER: int = 1
FUSION_FILTER: int = 2


class ImageFilterNode:
    """
    A class for defining a ROS node to filter ultrasound images.
    """

    def __init__(self, filter_type: int, visualizations_included: list, filtering_rate: float = -1,
                 debug_mode: bool = False, analysis_mode: bool = False):
        """
        Create a ROS node to filter raw ultrasound images and publish data about them.

        Parameters
        ----------
        filter_type
            selector to determine which kind of filter is associated with this node.

        visualizations_included
            a list of integers representing the visualizations to show for each image filtered.

        filtering_rate
            rate in Hz at which the image filter will attempt to filter images. Defaults to as fast as possible.

        debug_mode
            display graphics and additional print statements helpful in the debugging process.

        analysis_mode
            display the time key processes take to occur.
        """

        # -----------------------------
        # Define class attributes
        # -----------------------------

        # set parameters assisting in measuring and debugging code
        self.debug_mode = debug_mode
        self.analysis_mode = analysis_mode

        # Define flags and variables used within the class
        self.is_new_image_available = False     # flag showing when a new image to filter is available
        self.newest_image_data: ImageData = None           # for storing the data about the most recently sent image
        self.image_data: ImageData = None                  # for storing the data about the image to be filtered
        self.time_of_last_image_filtered = 0    # for storing when the first image was filtered
        self.filter_images = False              # for storing the current image filtering status
        self.temporary_visualization_block = False

        # check that a valid filtering rate was given
        try:
            1 / filtering_rate
        except ValueError:
            raise Exception("Filtering rate must be a non-zero value. A value of " + str(filtering_rate) +
                            " was given.")

        # save the filtering rate passed to the function
        self.filtering_rate: float = filtering_rate  # hz

        # Create the image filter used in this node and define a title for it.
        if filter_type == THRESHOLD_FILTER:
            self.image_filter = ImageFilterThreshold(debug_mode=self.debug_mode, analysis_mode=self.analysis_mode)

            # Define the title to use in the visualization
            visualization_title = "Threshold Filter"

        elif filter_type == GRABCUT_FILTER:
            self.image_filter = ImageFilterGrabCut(None, debug_mode=self.debug_mode, analysis_mode=self.analysis_mode,
                                                   image_crop_included=True,
                                                   image_crop_coordinates=[[171, 199], [530, 477]])

            # Define the title to use in the visualization
            visualization_title = "Grabcut Filter"
        else:
            raise Exception("Image filter type not recognized.")

        # store the visualization object to use in this node
        self.visualization = Visualization(IMG_CONTINUOUS, visualizations_included, visualization_title)

        # store an image positioning controller object to calculate the image centroid error
        self.image_positioning_controller = ImagePositioningController(debug_mode=self.debug_mode,
                                                                       analysis_mode=self.analysis_mode)

        # -----------------------------
        # Define ROS components
        # -----------------------------

        # initialize ros node
        init_node('image_filtering')

        # Create a subscriber to receive the ultrasound images
        self.raw_image_subscriber = Subscriber('/Clarius/US', Image, self.raw_image_callback)

        # Create a subscriber to receive the command to start and stop filtering images
        self.filter_images_subscriber = Subscriber('/command/filter_images', Bool, self.filter_images_callback)

        # Create a subscriber to receive the command to select the crop coordinates for the image
        self.select_crop_coordinates_subscriber = Subscriber('/command/select_crop_coordinates', Bool,
                                                             self.select_crop_coordinates_callback)

        # Create a subscriber to receive the command to create the grabcut filter mask
        self.generate_grabcut_mask_subscriber = Subscriber('/command/generate_grabcut_mask', Bool,
                                                           self.generate_grabcut_mask_callback)

        # Create a subscriber to receive the command to generate the threshold filter parameters
        self.generate_threshold_parameters_subscriber = Subscriber('/command/generate_threshold_parameters', Bool,
                                                                   self.generate_threshold_parameters_callback)

        # Create a publisher to publish the error of the centroid
        self.image_based_control_input_publisher = Publisher('/control_input/image_based', TwistStamped,
                                                             queue_size=1)

        # Create a publisher to publish if the thyroid is visible in the image
        self.is_thyroid_in_image_status_publisher = Publisher('/status/thyroid_shown', Bool, queue_size=1)

        # Create a publisher to publish if the thyroid is centered in the image
        self.is_thyroid_centered_status_publisher = Publisher('/status/thyroid_centered', Bool, queue_size=1)

        # Create a publisher to publish debugging status messages
        self.debug_status_messages_publisher = Publisher('/debug/status_messages', String, queue_size=1)

        # Create a publisher to publish the masked image
        # mask_publisher = rospy.Publisher('/image_data/mask', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the mask overlaid image
        # mask_overlay_publisher = rospy.Publisher('image_data/mask_overlay', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the thyroid contours
        # contour_publisher = rospy.Publisher('/image_data/contours', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the centroids of the thyroid
        # centroid_publisher = rospy.Publisher('/image_data/centroids', UInt32MultiArray, queue_size=1)

    #############################################################################
    # Define ROS callbacks
    #############################################################################

    def raw_image_callback(self, data: Image):
        """
        Regulates filtering rate and updates object parameters.
        """

        # record the current time for timing of processes
        start_of_process_time = time()

        try:
            """# Convert image using CvBridge
            # convert the image message to a cv2 data array
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')"""

            # Convert image using numpy tools
            cv_image = frombuffer(data.data, dtype=uint8)
            cv_image = reshape(cv_image, (data.height, data.width))

            # note the amount of time required to convert the image
            # start_of_process_time = self.display_process_timer(start_of_process_time, "Image conversion time")

            # Create new image data based on received image
            self.newest_image_data = ImageData(image_data=cv_image, image_color=COLOR_GRAY)

            # Crop and recolorize the newest image if an image crop is included
            if self.image_filter.image_crop_included:
                self.image_filter.crop_image(self.newest_image_data)
                self.image_filter.colorize_image(self.newest_image_data)

            # Visualize the image accordingly
            if self.debug_mode:
                if self.image_filter.image_crop_included:

                    if self.newest_image_data.cropped_image is not None:

                        self.visualization.visualize_images(self.newest_image_data, [SHOW_RECOLOR])

                    else:
                        self.visualization.visualize_images(self.newest_image_data, [SHOW_CROPPED])

                else:
                    self.visualization.visualize_images(self.newest_image_data, [SHOW_ORIGINAL])

        except CvBridgeError:
            print("Image error.")
            self.newest_image_data = None
            return

        # check if the image needs to be filtered
        time_since_last_image = time() - self.time_of_last_image_filtered

        if time_since_last_image > (1 / self.filtering_rate) and self.filter_images:
            # record the current time
            self.time_of_last_image_filtered = time()

            # Create new image data based on received image
            self.image_data = copy(self.newest_image_data)

            # note the amount of time required to create an image data object
            start_of_process_time = self.display_process_timer(start_of_process_time, "Image_data object creation time")

            # mark that a new image is available
            self.is_new_image_available = True

            # analyze the new image
            self.analyze_image()

            # note the amount of time required to analyze the image
            self.display_process_timer(start_of_process_time, "Image analysis time")

    def filter_images_callback(self, data: Bool):
        """
        Update filter_images_command parameter.
        """
        self.filter_images = data.data

    def select_crop_coordinates_callback(self, data: Bool):
        """
        When commanded, call on the image filter to crop the image.
        """

        # update status message
        self.publish_status("Crop command received")

        # If a new image data object exists
        if self.newest_image_data is not None:

            # update status message
            self.publish_status("Generating crop coordinates")

            # Generate the image crop coordinates
            self.image_filter.generate_crop_coordinates(self.newest_image_data)

        else:
            self.publish_status("No image available to crop")

    def generate_grabcut_mask_callback(self, data: Bool):
        """
        When commanded, call on the image filter to generate the previous mask.
        Only works for GrabCut image filters.
        """

        if self.newest_image_data is not None:
            if type(self.image_filter) is ImageFilterGrabCut:

                # Define default values for lists of points for background and foreground
                list_of_background_points = [(14, 194), (47, 14), (285, 7), (322, 154), (292, 148), (265, 135),
                                             (234, 128), (186, 131), (139, 135), (106, 144), (80, 161), (58, 179),
                                             (38, 198), (30, 218), (31, 245), (55, 262), (65, 275), (35, 271), (8, 270)]
                list_of_foreground_points = [(63, 213), (81, 195), (114, 174), (134, 166), (131, 186), (125, 203),
                                             (115, 222), (113, 235), (109, 250), (96, 246), (72, 231), (65, 228)]

                self.image_filter.generate_previous_mask_from_user_input(self.newest_image_data,
                                                                         list_of_background_points=list_of_background_points,
                                                                         list_of_foreground_points=list_of_foreground_points)
        else:
            self.debug_status_messages_publisher.publish(String("No image available from which "
                                                                "to create grabcut mask"))

    def generate_threshold_parameters_callback(self, data: Bool):
        """
        When commanded, call on the image filter to generate the ideal thresholding parameters.
        Only works for Threshold image filters.
        """
        if self.newest_image_data is not None:
            if type(self.image_filter) is ImageFilterThreshold:
                self.image_filter.generate_threshold_parameters(self.newest_image_data)
        else:
            self.debug_status_messages_publisher.publish(String("No image available from which to "
                                                                "generate threshold parameters"))

    #############################################################################
    # Define Helpers
    #############################################################################

    def analyze_image(self):
        """
        Analyzes image saved as image_data object in node and publishes data about it.

        If the thyroid is not found in the image, the method publishes empty data.
        """

        # save the start of the process time
        start_of_process_time = time()

        # filter the image
        self.image_filter.fully_filter_image(self.image_data)

        # find the contours in the mask
        self.image_data.generate_contours_in_image()

        # note the time required to fully filter the image
        start_of_process_time = self.display_process_timer(start_of_process_time, "Time to fully filter")

        # visualize the result of the image filtering
        self.visualization.visualize_images(self.image_data)

        # note the time required to visualize the image
        start_of_process_time = self.display_process_timer(start_of_process_time, "Time to visualize image")

        # Determine if the thyroid is present in the image
        thyroid_in_image = (len(self.image_data.contours_in_image) > 0 and
                            len(self.image_data.contours_in_image) == len(self.image_data.contour_centroids))

        # Publish if the thyroid is in the image
        self.is_thyroid_in_image_status_publisher.publish(Bool(thyroid_in_image))

        # note the time required to determine and publish the status of the thyroid in the image
        start_of_process_time = self.display_process_timer(start_of_process_time, "Thyroid status check time")

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

            """if self.debug_mode:
                # Create a temporary recolored image for debugging purposes
                temp_masked_result = cv2.cvtColor(self.image_data.original_image, cv2.COLOR_GRAY2RGB)

                # Add mask to original image
                temp_masked_result = cv2.drawContours(temp_masked_result, self.image_data.contours_in_image[0],
                                                      -1, (0, 255, 0), -1)

                # Add centroid to masked image
                temp_masked_result = cv2.circle(temp_masked_result, self.image_data.contour_centroids[0], 6, 
                                            (255, 0, 0), -1)

                # Add lines showing where centroid should be
                temp_masked_result = cv2.line(temp_masked_result, (0, 240), (640, 240), (0, 0, 255), 2)
                temp_masked_result = cv2.line(temp_masked_result, (320, 0), (320, 480), (0, 165, 255), 2)

                # note the time required to mark up the original image for display
                start_of_process_time = display_process_timer(start_of_process_time, "Markup Time", self.analysis_mode)

                # Show masked image
                cv2.imshow("Marked-up image", temp_masked_result)
                cv2.waitKey(1)

                # note the time required to update the visualization
                display_process_timer(start_of_process_time, "Display Time", self.analysis_mode)"""

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

    def display_process_timer(self, start_of_process_time, message) -> float:
        """
        Display the amount of time a process ran in milliseconds. Return the current time of the function call.

        Parameters
        ----------
        start_of_process_time : float
            Time the process being measured started as marked with a call to time().

        message: str
            Description to display with the measured time.
        """
        return display_process_timer(start_of_process_time, message, self.analysis_mode)

    def publish_status(self, message_to_publish: str):
        """
        Publish debugging messages to the status bar on the experiment control application.
        """
        self.debug_status_messages_publisher.publish(String(message_to_publish))


if __name__ == '__main__':
    # create node object
    filter_node = ImageFilterNode(filter_type=GRABCUT_FILTER, visualizations_included=[SHOW_ORIGINAL, SHOW_FOREGROUND],
                                  filtering_rate=10, debug_mode=False, analysis_mode=True)

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # spin until node is terminated
    # callback function handles image analysis
    spin()


        
