#!/usr/bin/env python3

"""
File containing VisualizationNode class definition and ROS running code.
"""

# TODO - Low - Improve comments in this file.
# TODO - Low - Fix visualization titling issues

# TODO - HIGH - Why is this not visualizing things properly
# TODO - HIGH - Why is th centroid showing up in the wrong spot (x and y are reversed)?

# Import ROS packages
from rospy import init_node, Subscriber, Rate, is_shutdown

# Import custom ROS messages and services
from thyroid_ultrasound_messages.msg import image_data_message

# Import custom functions, classes, and constants
from thyroid_ultrasound_imaging_support.Visualization.VisualizationConstants import *
from thyroid_ultrasound_imaging_support.Visualization.Visualization import Visualization
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData


class VisualizationNode:
    def __init__(self,
                 image_mode,
                 visualizations_included: list = None,
                 debug_mode=True,
                 analysis_mode=True):

        # Define visualization object ot use to visualize images showing the result of the image filter
        self.image_visualizer = Visualization(image_mode, [SHOW_ORIGINAL, SHOW_CROPPED,
                                                           SHOW_POST_PROCESSED_MASK, SHOW_CENTROIDS_ONLY,
                                                           SHOW_FOREGROUND])

        init_node('visualizer')

        # Define a subscriber to listen for raw images
        Subscriber('image_data/raw', image_data_message, self.raw_image_data_message_callback)

        # Define a subscriber to listen for cropped images
        Subscriber('image_data/cropped', image_data_message, self.cropped_image_data_message_callback)

        # Define a subscriber to listen for fully filtered images
        Subscriber('image_data/filtered', image_data_message,
                   self.filtered_image_data_message_callback)

        # noinspection PyTypeChecker
        self.image_to_visualize: ImageData = None

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
        if self.image_to_visualize is None:
            self.image_to_visualize = temp_image_to_visualize
        else:
            self.image_to_visualize.original_image = temp_image_to_visualize.original_image

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
        if self.image_to_visualize is None:
            self.image_to_visualize = temp_image_to_visualize
        else:
            self.image_to_visualize.cropped_image = temp_image_to_visualize.cropped_image

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
        if self.image_to_visualize is None:
            self.image_to_visualize = temp_image_to_visualize
        else:
            self.image_to_visualize.colorized_image = temp_image_to_visualize.colorized_image
            self.image_to_visualize.down_sampled_image = temp_image_to_visualize.down_sampled_image
            self.image_to_visualize.pre_processed_image = temp_image_to_visualize.pre_processed_image
            self.image_to_visualize.image_mask = temp_image_to_visualize.image_mask
            self.image_to_visualize.post_processed_mask = temp_image_to_visualize.post_processed_mask
            self.image_to_visualize.sure_foreground_mask = temp_image_to_visualize.sure_foreground_mask
            self.image_to_visualize.sure_background_mask = temp_image_to_visualize.sure_background_mask
            self.image_to_visualize.probable_foreground_mask = temp_image_to_visualize.probable_foreground_mask
            self.image_to_visualize.contours_in_image = temp_image_to_visualize.contours_in_image
            self.image_to_visualize.contour_centroids = temp_image_to_visualize.contour_centroids

    def publish_updated_image(self):
        if self.image_to_visualize is not None:
            self.image_visualizer.visualize_images(self.image_to_visualize)


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
