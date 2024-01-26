#!/usr/bin/env python3

"""
File containing code to receive raw ultrasound images and rebroadcast them as ImageData objects.
"""

# TODO - Dream - Add logging through BasicNode class

# Import standard ROS specific packages
from sensor_msgs.msg import Image

# Import custom ROS specific packages
from thyroid_ultrasound_messages.msg import image_data_message

# Import from custom packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageFilter.FilterConstants import COLOR_GRAY
from thyroid_ultrasound_imaging_support.ImageData.convert_image_message_to_array import convert_image_message_to_array
from thyroid_ultrasound_support.BasicNode import *


class ImageDataConverter(BasicNode):

    def __init__(self):
        """
        Creates a ROS node to convert image messages into ImageData messages.
        """

        # Call init of super class
        super().__init__()

        # Create a ROS node
        init_node('ImageDataConverter')

        # Create a publisher for the images
        self.raw_image_publisher = Publisher(IMAGE_RAW, image_data_message, queue_size=100)

        # Create a subscriber to listen to commands to start and stop publishing images
        Subscriber(IMAGE_SOURCE, Image, self.raw_image_callback)

    def raw_image_callback(self, data: Image) -> image_data_message:
        """
        Receives a raw image, converts it to an ImageData object, and then publishes that object.
        """

        # Convert the Image message to an image array
        image_array = convert_image_message_to_array(data)

        # Declare image color
        image_color = COLOR_GRAY

        # Create new ImageData object
        new_image_object = ImageData(image_data=image_array, image_title="Raw Image Converter",
                                     image_capture_time=data.header.stamp, image_color=image_color,)

        # Create a new image_data_message from ImageData object
        new_image_data_message = new_image_object.convert_to_message()

        # Publish the new_image_data_message
        self.raw_image_publisher.publish(new_image_data_message)

        # Return the image data message to allow for validation
        return new_image_data_message


if __name__ == '__main__':

    # Create a new object instance for converting the images
    ImageDataConverter()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # Let the program run indefinitely
    spin()
