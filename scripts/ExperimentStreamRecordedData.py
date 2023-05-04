#!/usr/bin/env python3

"""
File containing code to stream recorded data as ROS Topic.
"""

# Import ROS specific packages
from rospy import init_node, Publisher, Rate, is_shutdown, Subscriber
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Import standard packages
from os import listdir
from os.path import isdir, isfile
from cv2 import imread, cvtColor, COLOR_BGR2GRAY
from sys import stdout

# Define global variable to control image streaming
stream_images = False


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

    # Define the rate at which to publish the images
    rate = Rate(publishing_rate)

    # loop
    try:

        # Define an iterator for the list of images created
        ii = 0

        # Calculate the number of images to send
        num_images = len(created_objects)

        # While the node is shutdown and more images are available
        while not is_shutdown() and ii < num_images:

            if stream_images:

                # Recolor the image to grayscale
                bscan = cvtColor(created_objects[ii], COLOR_BGR2GRAY)

                # Publish the image as an image message
                us_pub.publish(CvBridge().cv2_to_imgmsg(bscan, encoding="passthrough"))

                # Increment the iterator
                ii = ii + 1

                # Print how many images have been sent
                stdout.write(f'Image {ii} of {num_images} sent.\r')
                stdout.flush()

                # Sleep until the next time
                rate.sleep()

    except KeyboardInterrupt:
        print('terminated by user')


def streaming_commands_callback(data: Bool):
    global stream_images
    stream_images = data.data

if __name__ == '__main__':

    # Define root path to folders where images are stored
    root_path = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/Test/Images/'

    # Define the file path and image offset number to use
    this_file_path = 'Series2'
    this_image_offset_number = 20

    # Define the publishing rate for these images
    this_publishing_rate = 1 / 10  # 1 image / 10 seconds

    # Call the main function
    main(root_path + this_file_path, this_image_offset_number)
