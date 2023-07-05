#!/usr/bin/env python3

"""
File containing VisualizationNode class definition and ROS running code.
"""

# Import ROS packages
from rospy import init_node, spin, Subscriber

# Import custom ROS messages and services
from thyroid_ultrasound_imaging.msg import image_data_message

# Import custom functions, classes, and constants
from thyroid_ultrasound_imaging_support.Visualization.VisualizationConstants import *
from thyroid_ultrasound_imaging_support.Visualization.Visualization import Visualization
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData


class VisualizationNode:
    def __init__(self,
                 image_mode,
                 visualizations_included: list,
                 debug_mode=True,
                 analysis_mode=True):

        # Define visualization object to use to visualize all image messages received
        self.visualizer = Visualization(image_mode, visualizations_included)

        init_node('visualizer')

        # Define a subscriber to listen for image_data_messages
        self.image_data_message_subscriber = Subscriber('image_data', image_data_message,
                                                        self.image_data_message_callback)

    def image_data_message_callback(self, message: image_data_message):
        """
        Converts the received image data message to an image data object and then visualizes it.

        Parameters
        ----------
        message
            An image_data_msg representing the images to visualize.
        """

        # Generate an image data object to visualize
        image_to_visualize = ImageData(image_data_msg=message)

        # Visualize the image data
        self.visualizer.visualize_images(image_to_visualize)


if __name__ == '__main__':
    # create node object
    visualization_node = VisualizationNode(IMG_CONTINUOUS,
                                           visualizations_included=[SHOW_ORIGINAL,
                                                                    SHOW_FOREGROUND,
                                                                    SHOW_CENTROIDS_ONLY], )

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # spin until node is terminated
    # callback function handles image analysis
    spin()
