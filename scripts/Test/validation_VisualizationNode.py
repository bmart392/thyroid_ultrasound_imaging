# TODO - MEDIUM - Properly format this file

from copy import copy

from cv2 import imshow, waitKey
from rospy import Rate
from Test.validation_constants import IMAGE_DATA_OBJECTS_PATH
from VisualizationNode import VisualizationNode
from thyroid_ultrasound_imaging_support.Visualization.VisualizationConstants import *
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData

# Create the visualization node to test
test_node = VisualizationNode(IMG_CONTINUOUS)

# Define the rate at which the image will be refreshed
rate = Rate(30)  # Hz

# Load a saved image data object
test_image_data = ImageData(image_data_location=IMAGE_DATA_OBJECTS_PATH + '2024-01-17_07-44-35-529788_image-data-object')

raw_image_data_object = copy(test_image_data)
raw_image_data_object.cropped_image = None
raw_image_data_object.colorized_image = None
raw_image_data_object.down_sampled_image = None
raw_image_data_object.pre_processed_image = None
raw_image_data_object.image_mask = None
raw_image_data_object.post_processed_mask = None
raw_image_data_object.sure_foreground_mask = None
raw_image_data_object.sure_background_mask = None
raw_image_data_object.probable_foreground_mask = None
raw_image_data_object.contours_in_image = []
raw_image_data_object.contour_centroids = []

cropped_image_data_object = copy(test_image_data)
cropped_image_data_object.colorized_image = None
cropped_image_data_object.down_sampled_image = None
cropped_image_data_object.pre_processed_image = None
cropped_image_data_object.image_mask = None
cropped_image_data_object.post_processed_mask = None
cropped_image_data_object.sure_foreground_mask = None
cropped_image_data_object.sure_background_mask = None
cropped_image_data_object.probable_foreground_mask = None
cropped_image_data_object.contours_in_image = []
cropped_image_data_object.contour_centroids = []

filtered_image_data_object = copy(test_image_data)

# Loop infinitely
while True:

    # Send it to the raw image callback
    test_node.raw_image_data_message_callback(raw_image_data_object.convert_to_message())

    # Send it to the cropped image callback
    test_node.cropped_image_data_message_callback(cropped_image_data_object.convert_to_message())

    # Send it to the filtered image callback
    test_node.filtered_image_data_message_callback(filtered_image_data_object.convert_to_message())

    # Visualize the images
    test_node.publish_updated_image()

    # sleep
    rate.sleep()

# Success?