#!/usr/bin/env python3

"""
File containing the RealTimeImageFiterNode class definition and ROS running code.
"""

# TODO - Dream - Create fusion filter of Grabcut & Threshold types. Combine using Beysian probability.

# Import standard packages
from numpy import frombuffer, reshape, uint8
from time import time
from copy import copy

# Import standard ROS packages
from sensor_msgs.msg import Image

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import image_data_message

# Import custom packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData

from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilterThreshold import ImageFilterThreshold
from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilterGrabCut import ImageFilterGrabCut

from thyroid_ultrasound_imaging_support.Controller.ImagePositioningController import ImagePositioningController
from thyroid_ultrasound_imaging_support.Visualization.display_process_timer import display_process_timer
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_imaging_support.ImageFilter.SegmentationError import *

# Define the types of image filters that could be used
THRESHOLD_FILTER: int = 0
GRABCUT_FILTER: int = 1
FUSION_FILTER: int = 2


class RealTimeImageFilterNode(BasicNode):
    """
    A class for defining a ROS node to filter ultrasound images.
    """

    def __init__(self, filter_type: int,
                 debug_mode: bool = False, analysis_mode: bool = False):
        """
        Create a ROS node to filter raw ultrasound images and publish data about them.

        Parameters
        ----------
        filter_type
            selector to determine which kind of filter is associated with this node.

        debug_mode
            display graphics and additional print statements helpful in the debugging process.

        analysis_mode
            display the time key processes take to occur.
        """

        # Call the init function of the basic node class to inherit functions and parameters
        super().__init__()

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
        self.temporary_visualization_block = False
        self.current_imaging_depth = 5.2  # cm

        # Define a list to store the images received by the node
        self.received_images = []
        self.max_images_to_store = 3

        # Set the basic name to attach to each image
        image_title = "Image Filter Node - "

        # Create the image filter used in this node and define a title for it.
        if filter_type == THRESHOLD_FILTER:
            self.image_filter = ImageFilterThreshold(debug_mode=self.debug_mode, analysis_mode=self.analysis_mode)

            # Define the title to use in the visualization
            image_title = image_title + "Threshold Filter"

        elif filter_type == GRABCUT_FILTER:
            self.image_filter = ImageFilterGrabCut(None, debug_mode=self.debug_mode, analysis_mode=self.analysis_mode,
                                                   down_sampling_rate=0.35
                                                   # down_sampling_rate=1/4  # originally 0.5
                                                   # image_crop_coordinates=[[171, 199], [530, 477]],
                                                   # image_crop_coordinates=[[585, 455], [639, 479]],

                                                   )

            # Define the title to use in the visualization
            image_title = image_title + "Grabcut Filter"
        else:
            raise Exception("Image filter type not recognized.")

        # Store the title to append to each image data
        self.image_title = image_title

        # store an image positioning controller object to calculate the image centroid error
        self.image_positioning_controller = ImagePositioningController(debug_mode=self.debug_mode,
                                                                       analysis_mode=self.analysis_mode)

        # Define a variable to store a new initialization mask given to the node
        self.new_initialization_mask = None

        # endregion
        #########################

        #######################
        # Define ROS components
        # region

        # initialize ros node
        init_node(REAL_TIME_IMAGE_FILTER)

        # Create a publisher to publish if the region of interest is visible in the image
        self.is_roi_in_image_status_publisher = Publisher(IMAGE_ROI_SHOWN, Bool, queue_size=1)

        # Create a publisher to publish the cropped ultrasound image
        self.cropped_image_publisher = Publisher(IMAGE_CROPPED, image_data_message, queue_size=1)

        # Create a publisher to publish the fully filtered ultrasound image
        self.filtered_image_publisher = Publisher(IMAGE_FILTERED, image_data_message, queue_size=1)

        # Define services for the node
        Service(RTS_UPDATE_IMAGE_CROP_COORDINATES, UpdateImageCropCoordinates, self.crop_coordinates_handler)
        Service(RTS_UPDATE_INITIALIZATION_MASK, UpdateInitializationMask, self.grabcut_initialization_mask_handler)
        Service(RTS_UPDATE_THRESHOLD_PARAMETERS, UpdateThresholdParameters, self.update_threshold_parameters_handler)
        Service(RTS_NEWEST_IMAGE_DATA, NewestImageData, self.retrieve_newest_image_data_handler)
        Service(RTS_SET_SEGMENTATION_PHASE, StringRequest, self.set_segmentation_phase_handler)
        Service(RTS_HAS_SEGMENTATION_STABILIZED, ActionRequest, self.has_segmentation_stabilized_handler)

        # Create a subscriber to receive the ultrasound images
        Subscriber(IMAGE_SOURCE, Image, self.raw_image_callback)

        # Create a subscriber to receive if the patient is in the image
        Subscriber(IMAGE_PATIENT_CONTACT, Bool, self.patient_contact_callback)

        # Create a subscriber top listen for the imaging depth of the US probe
        Subscriber(IMAGE_DEPTH, Float64, self.imaging_depth_callback)

        # endregion
        #######################

        # Save the current time as the last time an image was published
        self.time_of_last_publishing = Time.now()

        self.log_single_message('Node ready')

    ######################
    # Define ROS callbacks
    # region

    def raw_image_callback(self, msg: Image):
        """
        Updates the list of received images and places the newest message at the end of the list.
        """

        # Create new image data based on received image and add it to the list
        self.received_images.append(ImageData(image_msg=msg, imaging_depth=self.current_imaging_depth))

        # Remove the oldest image if the list is now too long
        if len(self.received_images) > self.max_images_to_store:
            self.received_images.pop(0)

    def imaging_depth_callback(self, msg: Float64):
        """Saves the received imaging depth."""
        self.current_imaging_depth = msg.data

    def crop_coordinates_handler(self, req: UpdateImageCropCoordinatesRequest):
        """
        When new image crop coordinates are available, set the image filter to use them.
        """

        # Tell the image filter that it is no longer ready to filter images
        self.image_filter.ready_to_filter = False

        # Set the image crop coordinates to use to crop the image
        self.image_filter.image_crop_coordinates = [
            [req.first_coordinate_x, req.first_coordinate_y],
            [req.second_coordinate_x, req.second_coordinate_y]
        ]

        # Clear the history of masks in the filter
        self.image_filter.previous_image_masks = []

        self.log_single_message('New crop coordinates received')

        return UpdateImageCropCoordinatesResponse(True, NO_ERROR)

    def grabcut_initialization_mask_handler(self, req: UpdateInitializationMaskRequest):
        """
        When the user has created a new initialization mask, update the previous image mask for the image filter.
        Only works for GrabCut image filters.
        """

        if type(self.image_filter) is ImageFilterGrabCut:
            # Update the flag for the GrabCut filter so that a new previous-image-mask is not generated
            # from the current image
            self.image_filter.do_not_create_new_previous_image_mask = True

            # update status message
            self.log_single_message("New initialization mask received", LONG)

            # Convert the initialization mask from message form to array form
            initialization_mask = frombuffer(req.previous_image_mask.data, dtype=uint8)
            self.new_initialization_mask = reshape(initialization_mask, (req.previous_image_mask.height,
                                                                         req.previous_image_mask.width))

            return UpdateInitializationMaskResponse(True, NO_ERROR)

        return UpdateInitializationMaskResponse(False, 'Wrong filter type')

    def update_threshold_parameters_handler(self, req: UpdateThresholdParametersRequest):
        """
        When the user has generated new threshold parameters, update the parameters stored in the image filter.
        Only works for Threshold image filters.
        """
        if type(self.image_filter) is ImageFilterThreshold:
            # update status message
            self.log_single_message("New threshold parameters received", LONG)

            # Update the parameters of the filter
            self.image_filter.thresholding_parameters = (req.lower_bound, req.upper_bound)

            # Update the status message
            self.log_single_message("New threshold parameters saved to the filter", LONG)

            return UpdateThresholdParametersResponse(True, NO_ERROR)

        return UpdateThresholdParametersResponse(False, 'Wrong filter type')

    def retrieve_newest_image_data_handler(self, req: NewestImageDataRequest):
        """Retrieves the data stored in the newest image data variable."""
        return NewestImageDataResponse(image_data=self.newest_image_data.convert_to_message())

    def set_segmentation_phase_handler(self, req: StringRequestRequest):
        """Sets the segmentation phase of the image filter"""
        status, msg = self.image_filter.update_segmentation_phase(req.value)
        self.log_single_message('New segmentation phase of ' + req.value + ' has been set')
        return StringRequestResponse(status, msg)

    def has_segmentation_stabilized_handler(self, req: ActionRequestRequest):
        """Checks if the segmentation has stabilized"""
        status = self.image_filter.has_segmentation_stabilized()
        return ActionRequestResponse(status, NO_ERROR)

    def patient_contact_callback(self, data: Bool):
        self.image_filter.is_in_contact_with_patient = data.data

    # endregion
    ######################

    ################
    # Define Helpers
    # region

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

    # endregion
    ################

    def main_loop(self):
        """
        Process the newest available image.
        """
        # Define the default status to publish
        new_status = None

        if len(self.received_images) > 0:

            # record the current time for timing of processes
            start_of_process_time = time()

            # Create new image data based on received image
            self.newest_image_data = copy(self.received_images.pop(-1))

            # Crop the newest image
            try:
                self.image_filter.crop_image(self.newest_image_data)

            # If a segmentation error occurs while trying to crop the image,
            except SegmentationError as caught_exception:

                # Skip the image cropping step until correct image crop coordinates have been given
                self.image_filter.image_crop_coordinates = None

                # Add a new message to the history of the error
                caught_exception.history.append(CROP_FAILURE + " in 'main_loop' function of RealTimeImageFilterNode.py")

                # Log the history of the error
                self.log_single_message(caught_exception.convert_history_to_string(), SHORT)

                # Call the cropping function again so that the code can continue properly
                self.image_filter.crop_image(self.newest_image_data)

            # Colorize the newest image
            self.image_filter.colorize_image(self.newest_image_data)

            # Update the title of the image
            self.newest_image_data.image_title = self.image_title

            # Set a new status for the node
            new_status = NOT_READY_TO_FILTER

            # If a new initialization mask exists for the image
            if self.new_initialization_mask is not None:

                # Update the status
                new_status = UPDATING_FILTER_INITIALIZATION_MASK

                # Save the mask to the image filter
                self.image_filter.update_previous_image_mask(self.new_initialization_mask)

                # Clear the history of masks in the filter
                self.image_filter.previous_image_masks = []

                # Clear the new initialization mask
                self.new_initialization_mask = None

                # update status message
                self.log_single_message("New initialization mask saved to the filter", LONG)

                # update the flag to allow image filtering to occur
                self.image_filter.ready_to_filter = True

            # If the filter is ready and the patient is in the image,
            if self.image_filter.ready_to_filter and self.image_filter.is_in_contact_with_patient:
                # Create new image data based on received image
                # self.image_data = copy(self.newest_image_data)

                # note the amount of time required to create an image data object
                start_of_analysis_time = self.display_process_timer(start_of_process_time,
                                                                    "Image_data object creation time")

                # mark that a new image is available
                self.is_new_image_available = True

                # save the start of the process time
                start_of_process_time = time()

                try:

                    # Filter the image
                    self.image_filter.filter_image(self.newest_image_data, override_existing_data=False)

                    # Update the status
                    new_status = ANALYZING_IMAGE

                    # find the contours in the mask
                    self.newest_image_data.generate_contours_in_image()

                    # find the centroids of each contour
                    self.newest_image_data.calculate_image_centroids()

                    # note the time required to fully filter the image
                    start_of_process_time = self.display_process_timer(start_of_process_time, "Time to fully filter")

                    # Publish if the region of interest is in the image
                    self.is_roi_in_image_status_publisher.publish(
                        Bool((len(self.newest_image_data.contours_in_image) > 0 and
                              len(self.newest_image_data.contours_in_image) == len(
                                    self.newest_image_data.contour_centroids))))

                    # note the time required to determine and publish the status of the thyroid in the image
                    start_of_process_time = self.display_process_timer(start_of_process_time,
                                                                       "Thyroid status check time")

                    # set that a new image has not been delivered yet
                    self.is_new_image_available = False

                    # note the amount of time required to analyze the image
                    self.display_process_timer(start_of_analysis_time, "Image analysis time")

                except SegmentationError as caught_exception:
                    # Note that the image filter can no longer filter images
                    self.image_filter.ready_to_filter = False

                    # Add to the context of the exception
                    caught_exception.history.append(
                        "Failed to filter image in 'analyze_image' in RealTimeImageFilterNode.py")

                    # Log that the error occurred
                    self.log_single_message(caught_exception.convert_history_to_string(), SHORT)

            # Otherwise, publish that the ROI is not in the image
            else:
                self.is_roi_in_image_status_publisher.publish(False)

            # publish the result of the image filtering
            self.filtered_image_publisher.publish(self.newest_image_data.convert_to_message())

        self.publish_node_status(new_status=new_status, delay_publishing=0.5, default_status=WAITING)


if __name__ == '__main__':
    # create node object
    filter_node = RealTimeImageFilterNode(filter_type=GRABCUT_FILTER,
                                          debug_mode=False, analysis_mode=True)

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    while not is_shutdown():
        # Run the main loop until the program terminates
        filter_node.main_loop()

    print("Node Terminated.")
