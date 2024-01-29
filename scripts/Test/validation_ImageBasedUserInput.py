"""
Contains the code for validating that the ImageBasedUserInput ui_node is successful.
"""

# Import standard python packages
from cv2 import cvtColor, COLOR_BGR2GRAY, COLOR_BGR2RGB, COLOR_GRAY2RGB, COLOR_GRAY2BGR, GC_PR_BGD, GC_BGD, GC_FGD, \
    imshow, waitKey, GC_PR_FGD
from numpy import zeros, uint8, array, save, load
from matplotlib.pyplot import show, subplots
from os.path import realpath

# Import standard ROS packages
from std_msgs.msg import Bool

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_image_files import \
    load_folder_of_image_files
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageData.convert_image_message_to_array import convert_image_message_to_array
from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilterGrabCut import ImageFilterGrabCut, COLOR_GRAY
from validation_constants import FOLDER_PATH, INITIALIZATION_FILE_NAME, CROP_FILE_NAME, GROUND_TRUTH_FILE_NAME, INITIALIZATION_INFORMATION_PATH

# Import custom ROS packages
from ImageBasedUserInput import ImageBasedUserInput

# Create the ui_node to be tested
ui_node = ImageBasedUserInput()

# Read in a list of images from a folder
image_start_index = 51
images_as_arrays = load_folder_of_image_files('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts'
                                              '/Test/Images/2023-11-29_19-14', image_start_index)

# Define the image to use for testing
image_for_test = images_as_arrays[0]

# Create an image data object for testing
image_data_object_for_test = ImageData(image_data=cvtColor(image_for_test, COLOR_BGR2GRAY),
                                       image_color=COLOR_GRAY)

# Create the image_filter to support it
image_filter = ImageFilterGrabCut()

# Define the image message to use for testing the crop functionality
image_msg_for_crop_test = ImageData(image_data=cvtColor(image_for_test, COLOR_BGR2GRAY)).convert_to_message()

# Give the ui_node the image to crop
ui_node.raw_image_callback(image_msg_for_crop_test)

# Add the crop action
ui_node.generate_crop_coordinates_callback(Bool(True))

# Select the crop coordinates
crop_coordinates_message = ui_node.main_loop()

# Give the crop coordinates to the image filter node
image_filter.image_crop_included = True
image_filter.image_crop_coordinates = [
    [crop_coordinates_message.first_coordinate_x, crop_coordinates_message.first_coordinate_y],
    [crop_coordinates_message.second_coordinate_x, crop_coordinates_message.second_coordinate_y]
]

# Crop the image
image_filter.crop_image(image_data_object_for_test)

# Colorize the image
image_filter.colorize_image(image_data_object_for_test)

# Give the ui_node the colorized image to use for generating the mask
ui_node.cropped_image_callback(image_data_object_for_test.convert_to_message())

# Add a generate mask action
ui_node.generate_grabcut_initialization_mask_callback(Bool(True))

# Generate the mask
result_mask_msg = ui_node.main_loop()

# Pull the mask out of the result message
result_mask = convert_image_message_to_array(result_mask_msg.previous_image_mask)

# Add a "generate ground truth" action
ui_node.generate_ground_truth_mask_callback(Bool(True))

# Generate the ground truth mask
ground_truth_mask_msg = ui_node.main_loop()

# Pull the ground truth mask out of the message
ground_truth_mask = convert_image_message_to_array(ground_truth_mask_msg.previous_image_mask)

# Save the crop coordinates and mask, if desired
if True:
    save(INITIALIZATION_INFORMATION_PATH + INITIALIZATION_FILE_NAME, result_mask)
    save(INITIALIZATION_INFORMATION_PATH + CROP_FILE_NAME, array(image_filter.image_crop_coordinates))
    save(INITIALIZATION_INFORMATION_PATH + GROUND_TRUTH_FILE_NAME, ground_truth_mask)

# Define an array for storing a recolored version of the mask
colored_result_mask = zeros(image_data_object_for_test.cropped_image.shape).tolist()

# Fill in the colored result array based on the values in the result array
for row in range(image_data_object_for_test.cropped_image.shape[0]):
    for col in range(image_data_object_for_test.cropped_image.shape[1]):
        if result_mask[row][col] == GC_BGD:
            colored_result_mask[row][col] = array([0, 0, 0], dtype=uint8)
        elif result_mask[row][col] == GC_PR_FGD:
            colored_result_mask[row][col] = array([0, 0, 15], dtype=uint8)
        elif result_mask[row][col] == GC_FGD:
            colored_result_mask[row][col] = array([0, 15, 0], dtype=uint8)

# Create a figure to plot the results
fig, axes = subplots(2, 2)
fig.suptitle('Image Based User Input Process', fontsize=36)

# Show the original image
axes[0][0].imshow(cvtColor(image_data_object_for_test.original_image, COLOR_GRAY2RGB))
axes[0][0].set_title('Original Image', fontsize=24)

# Show the cropped & colorized image
axes[0][1].imshow(cvtColor(image_data_object_for_test.colorized_image, COLOR_BGR2RGB))
axes[0][1].set_title('Cropped Image based on Selected Points', fontsize=24)

# Show the mask generated by the user
axes[1][0].imshow(cvtColor(result_mask * uint8(125), COLOR_GRAY2RGB))
axes[1][0].set_title('User Generated Mask', fontsize=24)

# Overlay the mask on to the original image
axes[1][1].imshow(cvtColor(array(colored_result_mask, dtype=uint8) + image_data_object_for_test.colorized_image,
                           COLOR_BGR2RGB))
axes[1][1].set_title('Mask Overlaid on Original Image', fontsize=24)

# Show the figure
show()
