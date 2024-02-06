#!/usr/bin/env python3

"""
File containing code to stream recorded data as ROS Topic.
"""

# TODO - Dream - Convert this into standard node format
# TODO - Dream - Add logging through BasicNode class
# TODO - Dream - Add ability to play images in reverse order
# TODO - Dream - Add proper try-cath error checking everywhere and incorporate logging into it
# TODO - Dream - Replace node name with constant from NodeNameConstants
# TODO - Dream - Add proper status publishing

# Import ROS specific packages
from rospy import init_node, Publisher, is_shutdown, Subscriber, Time
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Import standard packages
from cv2 import cvtColor, COLOR_BGR2GRAY
from sys import stdout
from time import time

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_image_files import \
    load_folder_of_image_files
from thyroid_ultrasound_support.TopicNames import *

# Define global variable to control image streaming
stream_images = False

# Define global iterator variable
ii = 0


def main(file_path: str, image_number_offset: int = None, publishing_rate: float = 1):

    # Pull out the images from the given folder location
    created_objects = load_folder_of_image_files(file_path, starting_index=image_number_offset)

    # Ensure stream_images references the global variable
    global stream_images

    # Create a ROS node
    init_node('ClariusPublisherSpoof', anonymous=True)

    # Create a publisher for the images
    us_pub = Publisher(IMAGE_SOURCE, Image, queue_size=100)

    # Create a subscriber to listen to commands to start and stop publishing images
    Subscriber(IMAGE_STREAMING_CONTROL, Bool, streaming_commands_callback)

    # Create a subscriber to listen to commands to restart publishing images
    Subscriber(IMAGE_STREAMING_RESTART, Bool, restart_streaming_command_callback)

    # Define the time to wait between publishing the images
    delay_time = 1 / publishing_rate  # seconds

    # Define the time at which the last image was published
    previous_publish_time = 0

    # loop
    try:

        # Ensure the iterator references the global variable
        global ii

        # Calculate the number of images to send
        num_images = len(created_objects)

        # While the node is shutdown and more images are available
        while not is_shutdown() and ii < num_images:

            if stream_images and (time() - previous_publish_time) > delay_time:

                # Recolor the image to grayscale
                bscan = cvtColor(created_objects[ii], COLOR_BGR2GRAY)

                # Generate an image message to publish
                resulting_image_message: Image = CvBridge().cv2_to_imgmsg(bscan, encoding="passthrough")

                # Register when the image was taken
                resulting_image_message.header.stamp = Time.now()

                # Publish the image
                us_pub.publish(resulting_image_message)

                # Capture the current time as the time of the last image publishing
                previous_publish_time = time()

                # Increment the iterator
                ii = ii + 1

                # Print how many images have been sent
                stdout.write(f'Image {ii} of {num_images} sent.\r')
                stdout.flush()

    except KeyboardInterrupt:
        print('terminated by user')


def streaming_commands_callback(data: Bool):
    global stream_images
    stream_images = data.data


def restart_streaming_command_callback(data: Bool):
    global ii
    ii = 0


if __name__ == '__main__':

    # Define root path to folders where images are stored
    root_path = '/home/ben/thyroid_ultrasound_data/testing_and_validation/raw_images/'

    # Define the file path and image offset number to use
    this_file_path = '2023-11-29_19-14'
    this_image_offset_number = 1

    # Define the publishing rate for these images
    this_publishing_rate = 20  # Hz

    # Call the main function
    main(root_path + this_file_path, this_image_offset_number)
