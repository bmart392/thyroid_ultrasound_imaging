#!/usr/bin/env python3

"""
File containing code to stream recorded data as ROS Topic.
"""
import cv2

# Import ROS specific packages
from rospy import init_node, Publisher, Rate, is_shutdown, Subscriber, Time
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Import standard packages
from os import listdir
from os.path import isdir, isfile
from cv2 import imread, cvtColor, COLOR_BGR2GRAY, imshow
from sys import stdout
from time import time

# Define global variable to control image streaming
stream_images = False

# Define global iterator variable
ii = 0


def main(file_path: str, image_number_offset: int = None, publishing_rate: float = 1):

    # Check if the file path to the images is valid.
    if not isdir(file_path):
        raise "File path is not valid."

    # Set the offset number to 0 if none is provided
    if image_number_offset is None:
        image_number_offset = 0

    # Get a list of the files in the directory
    file_names = listdir(file_path)

    # Create an empty list in which to store the image data objects
    created_objects: list = list([None]) * (len(file_names))

    # Iterate through the list of file names found to ensure that
    # the image data objects are added to the list in sequential order.
    for file_name in file_names:

        # Append the file path to the file name.
        file_name_with_path = file_path + '/' + file_name

        # Check that the file name is valid.
        if not isfile(file_name_with_path):
            raise "File name is not valid."

        # Add the next image to the list of images in the correct position.
        image_position = int(int(file_name[file_name.find("_") + 1: file_name.find(".")]) - image_number_offset)
        created_objects[image_position] = imread(file_name_with_path)

    # Ensure stream_images references the global variable
    global stream_images

    # Create a ROS node
    init_node('ClariusPublisherSpoof', anonymous=True)

    # Create a publisher for the images
    us_pub = Publisher('Clarius/US', Image, queue_size=100)

    # Create a subscriber to listen to commands to start and stop publishing images
    Subscriber('/command/image_streaming_control', Bool, streaming_commands_callback)

    # Create a subscriber to listen to commands to restart publishing images
    Subscriber('/command/restart_image_streaming', Bool, restart_streaming_command_callback)

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
    root_path = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/Test/Images/'

    # Define the file path and image offset number to use
    this_file_path = '2023-11-29_19-14'
    this_image_offset_number = 1

    # Define the publishing rate for these images
    this_publishing_rate = 1 / 10  # 1 image / 10 seconds

    # Call the main function
    main(root_path + this_file_path, this_image_offset_number)
