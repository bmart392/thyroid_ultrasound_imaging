#!/usr/bin/env python3

"""
File containing VisualizationNode class definition and ROS running code.
"""

# TODO - Dream - Add logging with the BasicNode class
# TODO - Dream - Add proper try-cath error checking everywhere and incorporate logging into it
# TODO - Dream - Add proper node status publishing
# TODO - Medium - Add in visualizations for non-real time image filtering that is separate from the real-time visualization framework
# TODO - Dream - Store visuals in a dictionary so that windows can be closed whenever a command is sent to hide them

# Import standard python packages
from copy import deepcopy
from cv2 import imshow, waitKey

# Import custom ROS messages and services
from thyroid_ultrasound_messages.msg import image_data_message, SkinContactLines

# Import custom functions, classes, and constants
from thyroid_ultrasound_imaging_support.Visualization.VisualizationConstants import *
from thyroid_ultrasound_imaging_support.Visualization.Visualization import Visualization
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_support.BasicNode import *

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
        self.images_to_visualize = {IMAGE_RAW: [None, [SHOW_ORIGINAL]],
                                    IMAGE_CROPPED: [None, [SHOW_CROPPED]],
                                    IMAGE_FILTERED: [None, [SHOW_BLUR, SHOW_FOREGROUND, SHOW_CENTROIDS_ONLY]],
                                    IMAGE_SKIN_APPROXIMATION: [None, [SHOW_SKIN_APPROXIMATION], None]}

        # Create the node
        init_node(VISUALIZER)

        # Define a subscriber to listen for raw images
        Subscriber(IMAGE_RAW, image_data_message, self.raw_image_data_message_callback)

        # Define a subscriber to listen for cropped images
        Subscriber(IMAGE_CROPPED, image_data_message, self.cropped_image_data_message_callback)

        # Define a subscriber to listen for fully filtered images
        Subscriber(IMAGE_FILTERED, image_data_message,
                   self.filtered_image_data_message_callback)

        # Define a subscriber to listen for the skin approximation lines
        Subscriber(IMAGE_SKIN_APPROXIMATION, SkinContactLines, self.skin_contact_approximation_callback)

    ###########################
    # Define callback functions
    # region
    def raw_image_data_message_callback(self, message: image_data_message):
        """
        Converts the received raw image data message to an image data object and then visualizes it.

        Parameters
        ----------
        message
            An image_data_msg representing the raw image to visualize.
        """

        # Generate a temporary image data object
        temp_image_to_visualize = ImageData(image_data_msg=message)

        # Copy the original image field into the data to visualize
        if self.images_to_visualize[IMAGE_RAW][IMAGE] is None:
            self.images_to_visualize[IMAGE_RAW][IMAGE] = temp_image_to_visualize
        else:
            self.images_to_visualize[IMAGE_RAW][IMAGE].original_image = temp_image_to_visualize.original_image

    def cropped_image_data_message_callback(self, message: image_data_message):
        """
        Converts the received cropped image data message to an image data object and then visualizes it.

        Parameters
        ----------
        message
            An image_data_msg representing the cropped image to visualize.
        """
        # Generate a temporary image data object
        temp_image_to_visualize = ImageData(image_data_msg=message)

        # Copy the cropped image field into the data to visualize
        if self.images_to_visualize[IMAGE_CROPPED][IMAGE] is None:
            self.images_to_visualize[IMAGE_CROPPED][IMAGE] = temp_image_to_visualize
        else:
            self.images_to_visualize[IMAGE_CROPPED][IMAGE].cropped_image = temp_image_to_visualize.cropped_image

    def filtered_image_data_message_callback(self, message: image_data_message):
        """
        Converts the received filtered image data message to an image data object and then visualizes it.

        Parameters
        ----------
        message
            An image_data_msg representing the filtered image to visualize.
        """

        temp_image_to_visualize = ImageData(image_data_msg=message)

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

    # endregion
    ###########################
    def publish_updated_image(self):
        """
        Display all available images.
        """

        # For each type of image in the data set
        for key in self.images_to_visualize.keys():

            # If the image exists
            if self.images_to_visualize[key][IMAGE] is not None:
                # Visualize it
                if key == IMAGE_SKIN_APPROXIMATION:
                    self.image_visualizer.visualize_images(
                        deepcopy(self.images_to_visualize[key][IMAGE]),  # Coping avoids issues with data modification
                        specific_visualizations=self.images_to_visualize[key][VISUALIZATIONS],
                        skin_approximation_parameters=self.images_to_visualize[key][SKIN_CONTACT_PARAMETERS])
                else:
                    self.image_visualizer.visualize_images(
                        deepcopy(self.images_to_visualize[key][IMAGE]),  # Coping avoids issues with data modification
                        specific_visualizations=self.images_to_visualize[key][VISUALIZATIONS])


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
