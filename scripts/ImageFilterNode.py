#!/usr/bin/env python3

"""
File containing ImageFiterNode class definition and ROS running code.
"""

# TODO - Medium - Create fusion filter of Grabcut & Threshold types. Combine using Beysian probability.
# TODO - Medium - Fix image data object titling issues
# Import standard ROS packages
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

# Import standard packages
from numpy import frombuffer, reshape, uint8
from time import time
from copy import copy

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import image_data_message, image_crop_coordinates, \
    initialization_mask_message, threshold_parameters

# Import custom packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData

from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilterThreshold import ImageFilterThreshold
from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilterGrabCut import ImageFilterGrabCut
from thyroid_ultrasound_imaging_support.ImageFilter.FilterConstants import COLOR_GRAY

from thyroid_ultrasound_imaging_support.Visualization.VisualizationConstants import *
from thyroid_ultrasound_imaging_support.Visualization.Visualization import Visualization

from thyroid_ultrasound_imaging_support.Controller.ImagePositioningController import ImagePositioningController
from thyroid_ultrasound_imaging_support.Visualization.display_process_timer import display_process_timer
from thyroid_ultrasound_support.BasicNode import *

# Define the types of image filters that could be used
THRESHOLD_FILTER: int = 0
GRABCUT_FILTER: int = 1
FUSION_FILTER: int = 2


class ImageFilterNode(BasicNode):
    """
    A class for defining a ROS node to filter ultrasound images.
    """

    def __init__(self, filter_type: int, visualizations_included: list,
                 debug_mode: bool = False, analysis_mode: bool = False):
        """
        Create a ROS node to filter raw ultrasound images and publish data about them.

        Parameters
        ----------
        filter_type
            selector to determine which kind of filter is associated with this node.

        visualizations_included
            a list of integers representing the visualizations to show for each image filtered.

        debug_mode
            display graphics and additional print statements helpful in the debugging process.

        analysis_mode
            display the time key processes take to occur.
        """

        # Call the init function of the basic node class to inherit functions and parameters
        super(BasicNode, self).__init__()

        #########################
        # Define class attributes
        # region

        # set parameters assisting in measuring and debugging code
        self.debug_mode = debug_mode
        self.analysis_mode = analysis_mode

        # Define flags and variables used within the class
        self.is_new_image_available = False  # flag showing when a new image to filter is available
        # noinspection PyTypeChecker
        self.newest_image_data: ImageData = None  # for storing the data about the most recently sent image
        # noinspection PyTypeChecker
        self.image_data: ImageData = None  # for storing the data about the image to be filtered
        self.time_of_last_image_filtered = 0  # for storing when the first image was filtered
        self.filter_images = False  # for storing the current image filtering status
        self.temporary_visualization_block = False

        # Define a list to store the images received by the node
        self.received_images = []
        self.max_images_to_store = 20

        # Create the image filter used in this node and define a title for it.
        if filter_type == THRESHOLD_FILTER:
            self.image_filter = ImageFilterThreshold(debug_mode=self.debug_mode, analysis_mode=self.analysis_mode)

            # Define the title to use in the visualization
            visualization_title = "Threshold Filter"

        elif filter_type == GRABCUT_FILTER:
            self.image_filter = ImageFilterGrabCut(None, debug_mode=self.debug_mode, analysis_mode=self.analysis_mode,
                                                   #down_sampling_rate=1/4  # originally 0.5
                                                   # image_crop_included=True,
                                                   # image_crop_coordinates=[[171, 199], [530, 477]],
                                                   # image_crop_coordinates=[[585, 455], [639, 479]],

                                                   )

            # Define the title to use in the visualization
            visualization_title = "Grabcut Filter"
        else:
            raise Exception("Image filter type not recognized.")

        # TODO Get rid of this since this node does not visualize things anymore
        # store the visualization object to use in this node
        self.visualization = Visualization(IMG_CONTINUOUS, visualizations_included, visualization_title)

        # store an image positioning controller object to calculate the image centroid error
        self.image_positioning_controller = ImagePositioningController(debug_mode=self.debug_mode,
                                                                       analysis_mode=self.analysis_mode)

        # endregion
        #########################

        #######################
        # Define ROS components
        # region

        # initialize ros node
        init_node('image_filtering')

        # Create a subscriber to receive the ultrasound images
        Subscriber(IMAGE_RAW, image_data_message, self.raw_image_callback)

        # Create a subscriber to receive the command to start and stop filtering images
        Subscriber(FILTER_IMAGES, Bool, self.filter_images_callback)

        # Create a subscriber to receive the command to select the crop coordinates for the image
        Subscriber(IMAGE_CROP_COORDINATES, image_crop_coordinates, self.crop_coordinates_callback)

        # Create a subscriber to receive the command to create the grabcut filter mask
        Subscriber(INITIALIZATION_MASK, initialization_mask_message, self.grabcut_initialization_mask_callback)

        # Create a subscriber to receive the command to generate the threshold filter parameters
        Subscriber(THRESHOLD_PARAMETERS, threshold_parameters, self.update_threshold_parameters_callback)

        # Create a subscriber to receive if the patient is in the image
        Subscriber(IMAGE_PATIENT_CONTACT, Bool, self.patient_contact_callback)

        # # Create a publisher to publish the error of the centroid
        # self.image_based_control_input_publisher = Publisher(RC_IMAGE_ERROR, TwistStamped,
        #                                                      queue_size=1)

        # Create a publisher to publish if the thyroid is visible in the image
        self.is_thyroid_in_image_status_publisher = Publisher('/status/thyroid_shown', Bool, queue_size=1)

        # Create a publisher to publish if the thyroid is centered in the image
        self.is_thyroid_centered_status_publisher = Publisher('/status/thyroid_centered', Bool, queue_size=1)

        # Create a publisher to publish debugging status messages
        self.debug_status_messages_publisher = Publisher('/debug/status_messages', String, queue_size=1)

        # Create a publisher to publish the cropped ultrasound image
        self.cropped_image_publisher = Publisher('image_data/cropped', image_data_message, queue_size=1)

        # Create a publisher to publish the fully filtered ultrasound image
        self.filtered_image_publisher = Publisher('image_data/filtered', image_data_message, queue_size=1)

        # Create a publisher to publish when the patient is in view in the image
        self.patient_contact_publisher = Publisher(IMAGE_PATIENT_CONTACT, Bool, queue_size=1)

        # TODO - Low - Clean-up all of these unused publishers.
        # Create a publisher to publish the masked image
        # mask_publisher = rospy.Publisher('/image_data/mask', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the mask overlaid image
        # mask_overlay_publisher = rospy.Publisher('image_data/mask_overlay', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the thyroid contours
        # contour_publisher = rospy.Publisher('/image_data/contours', UInt32MultiArray, queue_size=1)

        # Create a publisher to publish the centroids of the thyroid
        # centroid_publisher = rospy.Publisher('/image_data/centroids', UInt32MultiArray, queue_size=1)

        # endregion
        #######################

    ######################
    # Define ROS callbacks
    # region

    def raw_image_callback(self, data: image_data_message):
        """
        Updates the list of received images and places the newest message at the end of the list.
        """

        # Create new image data based on received image and add it to the list
        self.received_images.append(ImageData(image_data_msg=data))

        # Remove the oldest image if the list is now too long
        if len(self.received_images) > self.max_images_to_store:
            self.received_images.pop(0)

    def filter_images_callback(self, data: Bool):
        """
        Update filter_images_command parameter.
        """
        self.filter_images = data.data

    def crop_coordinates_callback(self, coordinates: image_crop_coordinates):
        """
        When new image crop coordinates are available, set the image filter to use them.
        """

        # Set the image filter to crop the image
        self.image_filter.image_crop_included = True

        # Set the image crop coordinates to use to crop the image
        self.image_filter.image_crop_coordinates = [
            [coordinates.first_coordinate_x, coordinates.first_coordinate_y],
            [coordinates.second_coordinate_x, coordinates.second_coordinate_y]
        ]

    def grabcut_initialization_mask_callback(self, data: initialization_mask_message):
        """
        When the user has created a new initialization mask, update the previous image mask for the image filter.
        Only works for GrabCut image filters.
        """

        # TODO - Low - Add comparison between image used to generate mask and current image to ensure mask is still good

        if type(self.image_filter) is ImageFilterGrabCut:

            # update the flag to allow image filtering to occur
            self.image_filter.ready_to_filter = True

            # update status message
            self.publish_status("New initialization mask received")

            # Convert the initialization mask from message form to array form
            initialization_mask = frombuffer(data.previous_image_mask.data, dtype=uint8)
            initialization_mask = reshape(initialization_mask, (data.previous_image_mask.height,
                                                                data.previous_image_mask.width))

            # Save the mask to the image filter
            self.image_filter.update_previous_image_mask(initialization_mask)

            # update status message
            self.publish_status("New initialization mask saved to the filter")

    def update_threshold_parameters_callback(self, data: threshold_parameters):
        """
        When the user has generated new threshold parameters, update the parameters stored in the image filter.
        Only works for Threshold image filters.
        """
        if type(self.image_filter) is ImageFilterThreshold:

            # update status message
            self.publish_status("New threshold parameters received")

            # Update the parameters of the filter
            self.image_filter.thresholding_parameters = (data.lower_bound, data.upper_bound)

            # Update the status message
            self.publish_status("New threshold parameters saved to the filter")

    def patient_contact_callback(self, data: Bool):
        self.image_filter.is_in_contact_with_patient = data.data

    # endregion
    ######################

    ################
    # Define Helpers
    # region

    def analyze_image(self):
        """
        Analyzes image saved as image_data object in node and publishes data about it.

        If the thyroid is not found in the image, the method publishes empty data.
        """

        # save the start of the process time
        start_of_process_time = time()

        # filter the image
        self.image_filter.filter_image(self.image_data)

        # find the contours in the mask
        self.image_data.generate_contours_in_image()

        # find the centroids of each contour
        self.image_data.calculate_image_centroids()

        # note the time required to fully filter the image
        start_of_process_time = self.display_process_timer(start_of_process_time, "Time to fully filter")

        # publish the result of the image filtering
        self.filtered_image_publisher.publish(self.image_data.convert_to_message())

        # note the time required to visualize the image
        start_of_process_time = self.display_process_timer(start_of_process_time, "Time to publish the filtered image")

        # Determine if the thyroid is present in the image
        thyroid_in_image = (len(self.image_data.contours_in_image) > 0 and
                            len(self.image_data.contours_in_image) == len(self.image_data.contour_centroids))

        # Publish if the thyroid is in the image
        self.is_thyroid_in_image_status_publisher.publish(Bool(thyroid_in_image))

        # note the time required to determine and publish the status of the thyroid in the image
        start_of_process_time = self.display_process_timer(start_of_process_time, "Thyroid status check time")

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

    # endregion
    ################

    def main_loop(self):
        """
        Process the newest available image.
        """

        if len(self.received_images) > 0:
            # record the current time for timing of processes
            start_of_process_time = time()

            # Create new image data based on received image
            self.newest_image_data = copy(self.received_images.pop(-1))

            # Crop and recolorize the newest image
            self.image_filter.crop_image(self.newest_image_data)
            self.image_filter.colorize_image(self.newest_image_data)

            # Publish this data so that it can be monitored before any filtering is completed
            self.newest_image_data.image_title = "Image Filter Node"
            self.cropped_image_publisher.publish(self.newest_image_data.convert_to_message())

            if self.filter_images and self.image_filter.ready_to_filter and \
                    self.image_filter.is_in_contact_with_patient:

                # Create new image data based on received image
                self.image_data = copy(self.newest_image_data)

                # note the amount of time required to create an image data object
                start_of_process_time = self.display_process_timer(start_of_process_time,
                                                                   "Image_data object creation time")

                # mark that a new image is available
                self.is_new_image_available = True

                # analyze the new image
                self.analyze_image()

                # note the amount of time required to analyze the image
                self.display_process_timer(start_of_process_time, "Image analysis time")


if __name__ == '__main__':
    # create node object
    filter_node = ImageFilterNode(filter_type=GRABCUT_FILTER, visualizations_included=[SHOW_ORIGINAL, SHOW_FOREGROUND,
                                                                                       SHOW_CENTROIDS_ONLY],
                                  debug_mode=False, analysis_mode=True)

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    while not is_shutdown():

        # Run the main loop until the program terminates
        filter_node.main_loop()

    print("Node Terminated.")
