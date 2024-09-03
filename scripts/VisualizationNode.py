#!/usr/bin/env python3

"""
File containing VisualizationNode class definition and ROS running code.
"""

# TODO - Medium - Add in visualizations for non-real time image filtering that is separate from
#  the real-time visualization framework
# TODO - Dream - Find a way to destroy the visualization windows when images are no longer being published to them

# Import standard ROS messages
from std_msgs.msg import Int8
from sensor_msgs.msg import Image

# Import standard python packages
from copy import deepcopy

# Import custom ROS messages and services
from thyroid_ultrasound_messages.msg import image_data_message, SkinContactLines

# Import custom functions, classes, and constants
from thyroid_ultrasound_imaging_support.Visualization.VisualizationConstants import *
from thyroid_ultrasound_imaging_support.Visualization.Visualization import Visualization
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_imaging_support.Controller.ImagePositioningControlConstants import IMAGE_CENTERING_OFFSET

# Define constants for use in the node
IMAGE: int = int(0)
VISUALIZATIONS: int = int(1)
SKIN_CONTACT_PARAMETERS: int = int(2)


class VisualizationNode(BasicNode):
    def __init__(self, image_mode):

        # Call the init of the super class
        super().__init__()

        # Define visualization object ot use to visualize images showing the result of the image filter
        self.image_visualizer = Visualization(image_mode, [])

        # noinspection PyTypeChecker
        # Define a place to store the Image Data object that will be visualized
        self.images_to_visualize = {IMAGE_RAW: [None, []],
                                    IMAGE_CROPPED: [None, []],
                                    IMAGE_FILTERED: [None, []],
                                    IMAGE_SKIN_APPROXIMATION: [None, [], None]}

        # Create the node
        init_node(VISUALIZER)

        # Define the subscriber to listen for the goal line location
        Subscriber(RC_IMAGE_CENTERING_SIDE, Int8, self.image_centering_side_callback)

        # Define a subscriber to listen for raw images
        Subscriber(IMAGE_SOURCE, Image, self.raw_image_data_message_callback)

        # Define a subscriber to listen for fully filtered images
        Subscriber(IMAGE_FILTERED, image_data_message,
                   self.filtered_image_data_message_callback)

        # Define a subscriber to listen for the skin approximation lines
        Subscriber(IMAGE_SKIN_APPROXIMATION, SkinContactLines, self.skin_contact_approximation_callback)

        # Define services for setting the visualizations
        for service_name in [VIS_STATUS_SHOW_ORIGINAL, VIS_STATUS_SHOW_CROPPED,
                             VIS_STATUS_SHOW_RECOLOR, VIS_STATUS_SHOW_BLUR,
                             VIS_STATUS_SHOW_RESULT_MASK, VIS_STATUS_SHOW_POST_PROCESSED_MASK,
                             VIS_STATUS_SHOW_SURE_FOREGROUND, VIS_STATUS_SHOW_SURE_BACKGROUND,
                             VIS_STATUS_SHOW_PROBABLE_FOREGROUND, VIS_STATUS_SHOW_INITIALIZATION_MASK,
                             VIS_STATUS_SHOW_CENTROIDS_ONLY, VIS_STATUS_SHOW_CENTROIDS_CROSS_ONLY,
                             VIS_STATUS_SHOW_MASK_CENTROIDS_CROSS_OVERLAY, VIS_STATUS_SHOW_FOREGROUND,
                             VIS_STATUS_SHOW_SKIN_APPROXIMATION, VIS_STATUS_SHOW_GRABCUT_USER_INITIALIZATION_0]:
            Service(service_name, StatusVisualization, self.set_visualization)

    ###########################
    # Define callback functions
    # region
    def image_centering_side_callback(self, msg: Int8):
        """
        Sets the location of the imaging goal.
        """
        self.image_visualizer.image_centering_goal_offset = msg.data * IMAGE_CENTERING_OFFSET

    def raw_image_data_message_callback(self, message: Image):
        """
        Converts the received raw image data message to an image data object and then visualizes it.

        Parameters
        ----------
        message
            An image_data_msg representing the raw image to visualize.
        """

        # Generate a temporary image data object
        temp_image_to_visualize = ImageData(image_msg=message, image_title='Ultrasound Probe')

        # Copy the original image field into the data to visualize
        if self.images_to_visualize[IMAGE_RAW][IMAGE] is None:
            self.images_to_visualize[IMAGE_RAW][IMAGE] = temp_image_to_visualize
        else:
            self.images_to_visualize[IMAGE_RAW][IMAGE].original_image = temp_image_to_visualize.original_image

    def filtered_image_data_message_callback(self, message: image_data_message):
        """
        Converts the received filtered image data message to an image data object and then visualizes it.

        Parameters
        ----------
        message
            An image_data_msg representing the filtered image to visualize.
        """

        temp_image_to_visualize = ImageData(image_data_msg=message)

        # Copy the cropped image field into the data to visualize
        if self.images_to_visualize[IMAGE_CROPPED][IMAGE] is None:
            self.images_to_visualize[IMAGE_CROPPED][IMAGE] = temp_image_to_visualize
        else:
            self.images_to_visualize[IMAGE_CROPPED][IMAGE].cropped_image = temp_image_to_visualize.cropped_image

        # Copy the relevant image fields into the data to visualize
        if self.images_to_visualize[IMAGE_FILTERED][IMAGE] is None:
            self.images_to_visualize[IMAGE_FILTERED][IMAGE] = temp_image_to_visualize
        else:
            self.images_to_visualize[IMAGE_FILTERED][IMAGE].colorized_image = temp_image_to_visualize.colorized_image
            self.images_to_visualize[IMAGE_FILTERED][
                IMAGE].down_sampled_image = temp_image_to_visualize.down_sampled_image
            self.images_to_visualize[IMAGE_FILTERED][
                IMAGE].pre_processed_image = temp_image_to_visualize.pre_processed_image
            self.images_to_visualize[IMAGE_FILTERED][IMAGE].image_mask = temp_image_to_visualize.image_mask
            self.images_to_visualize[IMAGE_FILTERED][
                IMAGE].post_processed_mask = temp_image_to_visualize.post_processed_mask
            self.images_to_visualize[IMAGE_FILTERED][
                IMAGE].sure_foreground_mask = temp_image_to_visualize.sure_foreground_mask
            self.images_to_visualize[IMAGE_FILTERED][
                IMAGE].sure_background_mask = temp_image_to_visualize.sure_background_mask
            self.images_to_visualize[IMAGE_FILTERED][
                IMAGE].probable_foreground_mask = temp_image_to_visualize.probable_foreground_mask
            self.images_to_visualize[IMAGE_FILTERED][
                IMAGE].contours_in_image = temp_image_to_visualize.contours_in_image
            self.images_to_visualize[IMAGE_FILTERED][
                IMAGE].contour_centroids = temp_image_to_visualize.contour_centroids

    def skin_contact_approximation_callback(self, message: SkinContactLines):
        """
        Pulls the skin contact parameters out of the message and saves them for use later.
        Parameters
        ----------
        message :
            The message containing the skin approximation parameters to visualize.
        """

        # If a raw image has already been saved
        if self.images_to_visualize[IMAGE_RAW][IMAGE] is not None:
            # Create a temporary data object to use to visualize the skin approximation result
            temp_image_to_visualize: ImageData = deepcopy(self.images_to_visualize[IMAGE_RAW][IMAGE])

            # Rename the temp image title
            temp_image_to_visualize.image_title = "Image Contact Balance"

            # Save the image to use in the visualization
            self.images_to_visualize[IMAGE_SKIN_APPROXIMATION][IMAGE] = temp_image_to_visualize

            # Save the skin approximation parameters
            self.images_to_visualize[IMAGE_SKIN_APPROXIMATION][SKIN_CONTACT_PARAMETERS] = {
                SKIN_APPROXIMATION_MODE: message.skin_approximation_mode,
                BEST_FIT_LEFT_LINE_A: message.best_fit_left_line_a,
                BEST_FIT_LEFT_LINE_B: message.best_fit_left_line_b,
                BEST_FIT_RIGHT_LINE_A: message.best_fit_right_line_a,
                BEST_FIT_RIGHT_LINE_B: message.best_fit_right_line_b,
                BEST_FIT_SHARED_LINE_A: message.best_fit_shared_line_a,
                BEST_FIT_SHARED_LINE_B: message.best_fit_shared_line_b,
            }

    def set_visualization(self, req: StatusVisualizationRequest):
        """Handles the request to update the list of visualizations."""

        # Find the correct dictionary category
        if req.visualization == SHOW_ORIGINAL:
            dict_category = IMAGE_RAW
        elif req.visualization == SHOW_CROPPED:
            dict_category = IMAGE_CROPPED
        elif any([req.visualization == option for option in [SHOW_RECOLOR, SHOW_BLUR, SHOW_MASK,
                                                             SHOW_POST_PROCESSED_MASK, SHOW_SURE_FOREGROUND,
                                                             SHOW_SURE_BACKGROUND, SHOW_PROBABLE_FOREGROUND,
                                                             SHOW_INITIALIZED_MASK, SHOW_CENTROIDS_ONLY,
                                                             SHOW_CENTROIDS_CROSS_ONLY,
                                                             SHOW_MASK_CENTROIDS_CROSS_OVERLAY,
                                                             SHOW_FOREGROUND, SHOW_GRABCUT_USER_INITIALIZATION_0]]):
            dict_category = IMAGE_FILTERED
        elif req.visualization == SHOW_SKIN_APPROXIMATION:
            dict_category = IMAGE_SKIN_APPROXIMATION
        else:
            self.log_single_message('Visualization requested was not recognized')
            return StatusVisualizationResponse(was_successful=False, message='Visualization not recognized')

        # Add the visualization if necessary
        if req.status:
            if req.visualization not in self.images_to_visualize[dict_category][VISUALIZATIONS]:
                self.images_to_visualize[dict_category][VISUALIZATIONS].append(req.visualization)
                self.log_single_message('New visualization of added')

        # Otherwise remove the visualization if necessary
        else:
            if req.visualization in self.images_to_visualize[dict_category][VISUALIZATIONS]:
                self.images_to_visualize[dict_category][VISUALIZATIONS].pop(
                    self.images_to_visualize[dict_category][VISUALIZATIONS].index(req.visualization))
                self.log_single_message('Existing visualization removed')

        return StatusVisualizationResponse(was_successful=True, message='')

    # endregion
    ###########################
    def publish_updated_image(self):
        """Display all available images."""

        # Define the local variable for the status
        new_status = None

        # For each type of image in the data set
        for key in self.images_to_visualize.keys():

            # Only set the status the first time
            if new_status is None:
                new_status = NO_IMAGES_AVAILABLE

            # If the image exists,
            if self.images_to_visualize[key][IMAGE] is not None:

                # Set the local skin approximation parameters only if visualizing the skin approximation
                if key == IMAGE_SKIN_APPROXIMATION:
                    local_skin_approximation_parameters = self.images_to_visualize[key][SKIN_CONTACT_PARAMETERS]
                else:
                    local_skin_approximation_parameters = None

                # Visualize the image
                self.image_visualizer.visualize_images(
                    deepcopy(self.images_to_visualize[key][IMAGE]),  # Copying avoids issues with data modification
                    specific_visualizations=self.images_to_visualize[key][VISUALIZATIONS],
                    skin_approximation_parameters=local_skin_approximation_parameters)

                # Set the appropriate status
                new_status = VISUALIZING_IMAGES

        self.publish_node_status(new_status=new_status, delay_publishing=0.5,
                                 default_status=NO_VISUALIZATIONS_ACTIVE)


if __name__ == '__main__':
    # create node object
    visualization_node = VisualizationNode(IMG_CONTINUOUS)

    # Set the publishing rate for updating the visualization
    publishing_rate = Rate(30)  # Hz

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # While the nod is running
    while not is_shutdown():
        # Publish the correct visualization
        visualization_node.publish_updated_image()

        # Then wait
        publishing_rate.sleep()
