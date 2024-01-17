"""
Contains the code for validating that the ImageBasedUserInput ui_node is successful.
"""

# Import standard python packages
from cv2 import cvtColor, COLOR_BGR2GRAY
from numpy import array, save

# Import standard ROS packages
from std_msgs.msg import Bool

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_image_files import \
    load_folder_of_image_files
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageData.convert_image_message_to_array import convert_image_message_to_array
from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilterGrabCut import ImageFilterGrabCut, COLOR_GRAY

# Import custom ROS packages
from ImageBasedUserInput import ImageBasedUserInput

# Define source of images
IMAGE_SOURCE = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/Test/' \
               'Experimentation/Experiment_2024-01-12/Images'

# Define save locations for crop coordinates and ground truths
CROP_COORDINATES_SAVE_LOCATION = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/Test/' \
               'Experimentation/Experiment_2024-01-12/CropCoordinates'
GROUND_TRUTHS_SAVE_LOCATION = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/Test/' \
               'Experimentation/Experiment_2024-01-12/GroundTruths'

# Define the image crop coordinate for the top left corner
top_left_corner_crop_coordinate = [0, 79]

# Create the ui_node to be tested
ui_node = ImageBasedUserInput()

# Create the image_filter to support it
image_filter = ImageFilterGrabCut()

# Read in a list of images from a folder
images_for_test, image_indexes_for_test = load_folder_of_image_files(IMAGE_SOURCE, return_image_numbers=True)

for image_for_test, image_index_for_test in zip(images_for_test, image_indexes_for_test):

    # Create an image data object for testing
    image_data_object_for_test = ImageData(image_data=cvtColor(image_for_test, COLOR_BGR2GRAY),
                                           image_color=COLOR_GRAY)

    # Generate the image specific crop coordinates
    image_specific_crop_coordinates = [
        [top_left_corner_crop_coordinate[0], top_left_corner_crop_coordinate[1]],
        [image_for_test.shape[1] - 1, image_for_test.shape[0] - 1]
    ]

    # Save the image crop coordinates with the correct name
    save(CROP_COORDINATES_SAVE_LOCATION + '/CropCoordinates_' + str(image_index_for_test) + '.npy',
         array(image_specific_crop_coordinates))

    # Give the crop coordinates to the image filter node
    image_filter.image_crop_included = True
    image_filter.image_crop_coordinates = image_specific_crop_coordinates

    # Crop the image
    image_filter.crop_image(image_data_object_for_test)

    # Colorize the image
    image_filter.colorize_image(image_data_object_for_test)

    # Give the ui_node the colorized image to use for generating the mask
    ui_node.cropped_image_callback(image_data_object_for_test.convert_to_message())

    # Add a "Generate ground truth" action
    ui_node.generate_ground_truth_mask_callback(Bool(True))

    # Generate the ground truth mask
    ground_truth_mask_msg = ui_node.main_loop()

    # Pull the ground truth mask out of the message
    ground_truth_mask = convert_image_message_to_array(ground_truth_mask_msg.previous_image_mask)

    # Save the ground truth mask
    save(GROUND_TRUTHS_SAVE_LOCATION + '/GroundTruth_' + str(image_index_for_test) + '.npy',
         array(ground_truth_mask))

print("All masks generated.")

