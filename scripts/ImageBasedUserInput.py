#!/usr/bin/env python3

"""
Contains all code for the ImageBasedUserInput node.
"""

# TODO - Dream - Allow the user to either select just the foreground or the foreground and the background of the image
# TODO - Dream - Stop having the whole window close after giving each input step when generating threshold parameters.

# Import ROS packages
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Import custom ROS specific packages
from thyroid_ultrasound_messages.msg import image_data_message, image_crop_coordinates, \
    initialization_mask_message, threshold_parameters
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_services.srv import *

# Import from standard packages
from copy import copy
from numpy import load, save, array
from tkinter.filedialog import askopenfilename, asksaveasfilename
from tkinter import Tk

# Import from custom packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageFilter.FilterConstants import COLOR_GRAY
from thyroid_ultrasound_imaging_support.UserInput.user_input_crop_coordinates import user_input_crop_coordinates
from thyroid_ultrasound_imaging_support.UserInput.user_input_polygon_points import user_input_polygon_points
from thyroid_ultrasound_imaging_support.UserInput.get_threshold_values_user_input import \
    get_threshold_values_from_user_input
from thyroid_ultrasound_imaging_support.Boundaries.create_convex_triangles_from_points import \
    create_convex_triangles_from_points
from thyroid_ultrasound_imaging_support.Boundaries.create_mask_array_from_triangles import \
    create_mask_array_from_triangles
from thyroid_ultrasound_imaging_support.Boundaries.create_previous_image_mask_array_from_triangles import \
    create_previous_image_mask_array_from_triangles

# Define possible actions for the node
GENERATE_CROP: int = 0
GENERATE_CROP_FROM_TEMPLATE: int = 4
GENERATE_INITIALIZATION: int = 1
GENERATE_PARAMETERS: int = 2
GENERATE_GROUND_TRUTH_MASK: int = 3


class ImageBasedUserInput(BasicNode):

    def __init__(self):

        # Add the call to the super class
        super().__init__()

        # Define a list to store the actions that need to be taken in the order they need to be taken
        self.actions = []

        # Create the node object
        init_node(IMAGE_BASED_USER_INPUT)

        # Define service proxies
        self.coordinate_service = ServiceProxy(RTS_UPDATE_IMAGE_CROP_COORDINATES, UpdateImageCropCoordinates)
        self.initialization_mask_service = ServiceProxy(RTS_UPDATE_INITIALIZATION_MASK, UpdateInitializationMask)
        self.threshold_service = ServiceProxy(RTS_UPDATE_THRESHOLD_PARAMETERS, UpdateThresholdParameters)
        self.retrieve_newest_image_data = ServiceProxy(RTS_NEWEST_IMAGE_DATA, NewestImageData)

        # Define services for the node
        Service(IB_UI_CROP_IMAGE_FROM_POINTS, BoolRequest, self.generate_crop_coordinates_handler)
        Service(IB_UI_CROP_IMAGE_FROM_TEMPLATE, BoolRequest, self.generate_crop_coordinates_from_template_handler)
        Service(IB_UI_IDENTIFY_THYROID_FROM_POINTS, BoolRequest, self.generate_grabcut_initialization_mask_handler)

        # Define a subscriber to listen for the raw images
        # Subscriber(IMAGE_SOURCE, Image, self.raw_image_callback)

        # Define a subscriber to listen for the cropped images
        # Subscriber(IMAGE_CROPPED, image_data_message, self.cropped_image_callback)

        # Note when
        self.time_of_last_analysis = Time.now()
        self.log_single_message('Node ready')

    def main_loop(self, local_image_data: ImageData = None):

        # If an action needs to occur
        if len(self.actions) > 0:
            # Pop the next action out of the list
            next_action = self.actions.pop(0)

            # If the next action is to generate a crop from a template
            if next_action == GENERATE_CROP_FROM_TEMPLATE:

                self.publish_node_status(new_status=LOADING_CROP_COORDINATES)

                # Create a window that will be used to ask the user
                root = Tk()

                self.log_single_message('Waiting for user to select coordinates file')

                # Define the path to the stored file
                template_path: str = askopenfilename(
                    initialdir='/home/ben/thyroid_ultrasound_data/testing_and_validation/saved_templates',
                    title="Select cropping template to load.")

                # Close the window
                root.destroy()

                # Ensure that a file was selected and that it has the right extension
                if len(template_path) > 0 and template_path[-4:] == '.npy':

                    self.log_single_message('Coordinates saved at ' + template_path + ' were selected')

                    saved_coordinates = load(template_path)

                    # Create a message to publish the result
                    result_msg = image_crop_coordinates()

                    # Fill in the message with the proper data
                    result_msg.first_coordinate_x = saved_coordinates[0][0]
                    result_msg.first_coordinate_y = saved_coordinates[0][1]
                    result_msg.second_coordinate_x = saved_coordinates[1][0]
                    result_msg.second_coordinate_y = saved_coordinates[1][1]

                    try:
                        # Publish the response
                        self.coordinate_service(saved_coordinates[0][0],
                                                saved_coordinates[0][1],
                                                saved_coordinates[1][0],
                                                saved_coordinates[1][1])
                        self.log_single_message('New coordinates were sent')

                    except ServiceException:
                        self.log_single_message('New coordinates were not sent')

                    # Return the resulting message
                    return result_msg

                else:
                    self.log_single_message('Invalid path was selected')
                    return

            # Get image data from the image filter node to complete the remaining actions
            try:
                local_image_data = ImageData(image_data_msg=self.retrieve_newest_image_data().image_data)

            # If the service is not running and data was not passed into the function, simply do nothing
            except ServiceException:
                if local_image_data is None:
                    self.log_single_message('Service for retrieving image is not running')
                    return

            # If the next action is to crop the image,
            if next_action == GENERATE_CROP:

                self.publish_node_status(GENERATING_CROP)
                self.log_single_message('Ready for user to select the coordinates')

                # Allow the user to define the crop coordinates
                result_list = user_input_crop_coordinates(image_data=local_image_data)

                self.log_single_message('User selected crop coordinates')

                # Create a message to publish the result
                result_msg = image_crop_coordinates()

                # Fill in the message with the proper data
                result_msg.first_coordinate_x = result_list[0][0]
                result_msg.first_coordinate_y = result_list[0][1]
                result_msg.second_coordinate_x = result_list[1][0]
                result_msg.second_coordinate_y = result_list[1][1]

                # Publish the response
                self.coordinate_service(result_list[0][0], result_list[0][1],
                                        result_list[1][0], result_list[1][1])

                self.log_single_message('New coordinates sent')

                # Create a window that will be used to ask the user
                root = Tk()

                # Check if the user would like to save the cropping
                new_template_path = asksaveasfilename(
                    confirmoverwrite=True,
                    defaultextension='.npy',
                    initialdir='/home/ben/thyroid_ultrasound_data/testing_and_validation/saved_templates',
                    title="Select location to save cropping template.")

                # Close the window
                root.destroy()

                # If a path was selected, save the file
                if len(new_template_path) > 0:
                    save(new_template_path, array(result_list))

                    self.log_single_message('New coordinates saved at ' + new_template_path)

                # Return the resulting message
                return result_msg

            # If the next action is to generate a segmentation initialization,
            elif next_action == GENERATE_INITIALIZATION:

                self.publish_node_status(new_status=GENERATING_INITIALIZATION)
                self.log_single_message('Creating initial mask')

                # Define variables to store the selected points
                list_of_background_points = None
                list_of_foreground_points = None

                self.log_single_message('Waiting for point selection to finish')

                # Capture the background of the image from the user
                """list_of_points_for_background_polygon = user_input_polygon_points(
                    local_image_data,
                    "background",
                    display_result=True,
                    list_of_points=list_of_background_points)"""

                # Capture the foreground of the image from the user
                list_of_points_for_foreground_polygon = user_input_polygon_points(
                    local_image_data,
                    "region of interest",
                    display_result=True,
                    list_of_points=list_of_foreground_points)

                self.log_single_message('Foreground points selected')

                # Convert the points of the background and foreground polygons to triangles
                """list_of_background_triangles = create_convex_triangles_from_points(
                    list_of_points_for_background_polygon)"""
                list_of_foreground_triangles = create_convex_triangles_from_points(
                    list_of_points_for_foreground_polygon)

                # Generate the previous image mask using the triangles selected by the user
                initialization_mask = create_previous_image_mask_array_from_triangles(
                    [],  # list_of_background_triangles,
                    list_of_foreground_triangles,
                    local_image_data.colorized_image.shape[:2])

                self.log_single_message('Mask generated')

                # Create a CV bridge to convert the initialization mask to a message
                bridge = CvBridge()

                # Convert the initialization mask and save it to the message
                image_message_for_initialization_mask = bridge.cv2_to_imgmsg(initialization_mask)

                # Create a previous_image_mask_message to save the result the user of the user input in
                result_message = initialization_mask_message()

                # Fill in each field of the result message
                result_message.image_data = local_image_data.convert_to_message()
                result_message.previous_image_mask = image_message_for_initialization_mask

                try:
                    # Publish the result message
                    self.initialization_mask_service(local_image_data.convert_to_message(),
                                                     image_message_for_initialization_mask)
                    self.log_single_message('Mask sent')

                except ServiceException:
                    self.log_single_message('Mask not sent')

                # Return the resulting message
                return result_message

            # If the next action is to generate the parameters for a thresholding filter,
            elif next_action == GENERATE_PARAMETERS:

                self.publish_node_status(GENERATING_THRESHOLD)

                self.log_single_message('Ready to select threshold parameters')

                # Generate thresholding parameters
                result_parameters = get_threshold_values_from_user_input(local_image_data,
                                                                         num_standard_deviations=1.75)

                self.log_single_message('Threshold parameters selected')

                # Create a new message to send the resulting parameters
                result_message = threshold_parameters()

                # Save the individual parameters into the message
                result_message.lower_bound = result_parameters[0]
                result_message.upper_bound = result_parameters[1]

                # Publish the result of the threshold generation
                self.threshold_service(result_parameters[0], result_parameters[1])

                self.log_single_message('Threshold parameters selected')

                # Return the resulting message
                return result_message

            # If the next action is to generate a ground truth mask,
            elif next_action == GENERATE_GROUND_TRUTH_MASK:

                self.publish_node_status(GENERATING_GROUND_TRUTH)

                self.log_single_message('Ready to select points')

                # Define the list to store the points that define the foreground
                list_of_foreground_points = None

                # Capture the foreground of the image from the user
                list_of_points_for_foreground_polygon = user_input_polygon_points(
                    local_image_data,
                    "foreground ground truth",
                    display_result=True,
                    list_of_points=list_of_foreground_points)

                self.log_single_message('Points selected by user')

                # Convert the points of the foreground polygons to triangles
                list_of_foreground_triangles = create_convex_triangles_from_points(
                    list_of_points_for_foreground_polygon)

                # Generate the ground-truth mask using the triangles selected by the user
                ground_truth_mask = create_mask_array_from_triangles(
                    list_of_foreground_triangles,
                    local_image_data.colorized_image.shape[:2])

                self.log_single_message('Mask generated')

                # Create a CV bridge to convert the ground-truth mask to a message
                bridge = CvBridge()

                # Convert the ground-truth mask and save it to the message
                image_message_for_ground_truth_mask = bridge.cv2_to_imgmsg(ground_truth_mask)

                # Create a previous_image_mask_message to save the result the user of the user input in
                result_message = initialization_mask_message()

                # Fill in each field of the result message
                result_message.image_data = local_image_data.convert_to_message()
                result_message.previous_image_mask = image_message_for_ground_truth_mask

                # Publish the result message
                self.initialization_mask_service(local_image_data.convert_to_message(),
                                                 image_message_for_ground_truth_mask)

                self.log_single_message('Mask sent')

                # Return the resulting message
                return result_message

            else:
                raise Exception("Action type was not recognized.")

        self.publish_node_status(new_status=None, delay_publishing=0.5, default_status=WAITING)

    def generate_crop_coordinates_handler(self, req: BoolRequestRequest):
        if req.value:
            self.actions.append(GENERATE_CROP)
            self.log_single_message('Generate crop requested')
        return BoolRequestResponse(True, NO_ERROR)

    def generate_crop_coordinates_from_template_handler(self, req: BoolRequestRequest):
        if req.value:
            self.actions.append(GENERATE_CROP_FROM_TEMPLATE)
            self.log_single_message('Load crop requested')
        return BoolRequestResponse(True, NO_ERROR)

    def generate_grabcut_initialization_mask_handler(self, req: BoolRequestRequest):
        if req.value:
            self.actions.append(GENERATE_INITIALIZATION)
            self.log_single_message('Initial mask requested')
        return BoolRequestResponse(True, NO_ERROR)

    def generate_ground_truth_mask_callback(self, data: Bool):
        if data:
            self.actions.append(GENERATE_GROUND_TRUTH_MASK)
            self.log_single_message('Ground truth requested')


if __name__ == '__main__':
    # Create a new object instance for converting the images
    node = ImageBasedUserInput()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # While the node is running
    while not is_shutdown():
        # Let the program run indefinitely
        node.main_loop()

    print("Node terminated.")
