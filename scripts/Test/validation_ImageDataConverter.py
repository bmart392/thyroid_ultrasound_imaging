"""
Contains the code for validating that the ImageDataConverter node is successful.
"""

# Import standard python packages
from cv2 import imshow, waitKey
from numpy import sum

# Import custom python packages
from thyroid_ultrasound_imaging_support.Visualization.stitch_image_arrays import stitch_image_arrays
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_image_files import \
    load_folder_of_image_files
from thyroid_ultrasound_imaging_support.ImageData.convert_array_to_image_message import convert_array_to_image_message

# Import custom ROS packages
from Test.Archive.ImageDataConverter import *


# Define the node to test
test_node = ImageDataConverter()

# Read in a list of images from a folder
image_start_index = 1
images_as_arrays = load_folder_of_image_files('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts'
                                                 '/Test/Images/2023-11-29_19-14', image_start_index)

# Build a list of Image messages from the list of image arrays
images_as_messages = []
for image_array in images_as_arrays:
    images_as_messages.append(convert_array_to_image_message(image_array))

# Set the rate at which to display the images to the user
rate = Rate(30)  # Hz

# Define a variable to save the error between the two images
running_error_sum = 0

# For each image array and image message generated from that array
for image_array, image_message, image_number in zip(images_as_arrays, images_as_messages, range(len(images_as_arrays))):

    # Convert the image message to an image data message
    converted_image_message = test_node.raw_image_callback(image_message)

    # Create an image data object from that message
    converted_image_object = ImageData(image_data_msg=converted_image_message)

    # Stitch the two images together side by side
    array_to_show = stitch_image_arrays([image_array, converted_image_object.original_image])

    # Calculate the running error sum
    running_error_sum = running_error_sum + sum(image_array - converted_image_object.original_image)

    # Show the image to the user
    imshow("Slice Comparison: Original vs. Converted", array_to_show)
    waitKey(1)

    # Wait to show the next image
    rate.sleep()

# Display the total error accumulated
print("Total pixel based error between images was: " + str(running_error_sum))

