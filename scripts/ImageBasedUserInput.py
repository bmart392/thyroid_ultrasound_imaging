#!/usr/bin/env python3

# TODO - Medium - Properly comment this file.

# Import ROS packages
from rospy import init_node, Subscriber, Publisher, is_shutdown
from std_msgs.msg import Bool
from cv_bridge import CvBridge

# Import custom ROS specific packages
from thyroid_ultrasound_imaging.msg import image_data_message, image_crop_coordinates, \
    initialization_mask_message, threshold_parameters

# Import from standard packages
from copy import copy

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

GENERATE_CROP: int = 0
GENERATE_INITIALIZATION: int = 1
GENERATE_PARAMETERS: int = 2


class ImageBasedUserInput:

    def __init__(self):

        # Create the node object
        init_node('ImageBasedUserInput')

        # Define a subscriber to listen for the raw images
        Subscriber('image_data/raw', image_data_message, self.raw_image_callback)

        # Define a subscriber to listen for the cropped images
        Subscriber('image_data/cropped', image_data_message, self.cropped_image_callback)

        # Define a subscriber to listen for commands to crop the image
        Subscriber('command/generate_new_image_cropping', Bool, self.generate_crop_coordinates_callback)

        # Define a subscriber to listen for commands to generate the grabcut mask
        Subscriber('command/identify_thyroid_from_points', Bool, self.generate_grabcut_initialization_mask_callback)

        # Define a subscriber to listen for commands to generate the threshold filter parameters
        Subscriber('command/generate_threshold_parameters', Bool, self.generate_threshold_parameters_callback)

        # Create a publisher to publish the resulting coordinates from the user image cropping
        self.coordinate_publisher = Publisher('/ib_ui/image_crop_coordinates', image_crop_coordinates, queue_size=1)

        # Create a publisher to publish the resulting mask from the user image mask generation
        self.initialization_mask_publisher = Publisher('/ib_ui/initialization_mask', initialization_mask_message,
                                                       queue_size=1)

        # Create a publisher to publish the results of the user thresholding
        self.threshold_publisher = Publisher('/ib_ui/threshold_parameters', threshold_parameters,
                                             queue_size=1)

        # Define a place to store the image that will be cropped
        # noinspection PyTypeChecker
        self.image_to_crop: ImageData = None

        # Define a place to store the image that will be used to generate the initial mask and
        # setting the thresholding parameters
        # noinspection PyTypeChecker
        self.image_for_mask_and_threshold: ImageData = None

        # Define a list to store the actions that need to be taken in the order they need to be taken
        self.actions = []

    def main_loop(self):

        # While the node is running
        while not is_shutdown():

            # If an action needs to occur
            if len(self.actions) > 0:

                # Pop the next action out of the list
                next_action = self.actions.pop(0)

                if next_action == GENERATE_CROP:

                    # TODO - Medium - Figure out why this user input has the colors on the image weird.
                    # TODO - Low - Stop having the whole window close after giving each input step.

                    # Check that there is an image to crop before continuing
                    if self.image_to_crop is not None:
                        # Make a copy of the image to ensure that updates to the class
                        # parameter do not break the function
                        local_image_to_crop = copy(self.image_to_crop)

                        # Allow the user to define the crop coordinates
                        result_list = user_input_crop_coordinates(image_data=local_image_to_crop)

                        # Create a message to publish the result
                        result_msg = image_crop_coordinates()

                        # Fill in the message with the proper data
                        result_msg.first_coordinate_x = result_list[0][0]
                        result_msg.first_coordinate_y = result_list[0][1]
                        result_msg.second_coordinate_x = result_list[1][0]
                        result_msg.second_coordinate_y = result_list[1][1]

                        # Publish the response
                        self.coordinate_publisher.publish(result_msg)

                elif next_action == GENERATE_INITIALIZATION:

                    # TODO - Low - What if it did the expand thing so that you only had to select the foreground?
                    # TODO - Low - Stop having the whole window close after giving each input step.

                    # Check that there is an image to generate the initial mask from before continuing
                    if self.image_for_mask_and_threshold is not None:
                        # Make a copy of the image to ensure that updates to the class
                        # parameter do not break the function
                        local_image_to_generate_mask_from = copy(self.image_for_mask_and_threshold)

                        # Define default values for lists of points for the background and foreground of the image
                        # list_of_background_points = [(14, 194), (47, 14), (285, 7), (322, 154), (292, 148), (265, 135),
                        #                              (234, 128), (186, 131), (139, 135), (106, 144), (80, 161), (58, 179),
                        #                              (38, 198), (30, 218), (31, 245), (55, 262), (65, 275), (35, 271), (8, 270)]
                        # list_of_foreground_points = [(63, 213), (81, 195), (114, 174), (134, 166), (131, 186), (125, 203),
                        #                              (115, 222), (113, 235), (109, 250), (96, 246), (72, 231), (65, 228)]
                        list_of_background_points = None
                        list_of_foreground_points = None

                        # Capture the background of the image from the user
                        list_of_points_for_background_polygon = user_input_polygon_points(
                            local_image_to_generate_mask_from,
                            "background",
                            display_result=True,
                            list_of_points=list_of_background_points)

                        # Capture the foreground of the image from the user
                        list_of_points_for_foreground_polygon = user_input_polygon_points(
                            local_image_to_generate_mask_from,
                            "foreground",
                            display_result=True,
                            list_of_points=list_of_foreground_points)

                        # Convert the points of the background and foreground polygons to triangles
                        list_of_background_triangles = create_convex_triangles_from_points(
                            list_of_points_for_background_polygon)
                        list_of_foreground_triangles = create_convex_triangles_from_points(
                            list_of_points_for_foreground_polygon)

                        # Generate the previous image mask using the triangles selected by the user
                        initialization_mask = create_mask_array_from_triangles(list_of_background_triangles,
                                                                               list_of_foreground_triangles,
                                                                               local_image_to_generate_mask_from.cropped_image.shape[
                                                                               :2])

                        # Create a CV bridge to convert the initialization mask to a message
                        bridge = CvBridge()

                        # Create a new image message object to send the initialization mask
                        #image_message_for_initialization_mask = Image()

                        # Convert the initialization mask and save it to the message
                        image_message_for_initialization_mask = bridge.cv2_to_imgmsg(initialization_mask)

                        # Create a previous_image_mask_message to save the result the user of the user input in
                        result_message = initialization_mask_message()

                        # Fill in each field of the result message
                        result_message.image_data = local_image_to_generate_mask_from.convert_to_message()
                        result_message.previous_image_mask = image_message_for_initialization_mask

                        # Publish the result message
                        self.initialization_mask_publisher.publish(result_message)

                elif next_action == GENERATE_PARAMETERS:

                    # TODO - Low - Stop having the whole window close after giving each input step.

                    # Check that there is an image to generate the threshold parameters from before continuing
                    if self.image_for_mask_and_threshold is not None:
                        # Make a copy of the image to ensure that updates to the class
                        # parameter do not break the function
                        local_image_to_generate_threshold_from = copy(self.image_for_mask_and_threshold)

                        # Generate thresholding parameters
                        result_parameters = get_threshold_values_from_user_input(local_image_to_generate_threshold_from,
                                                                                 num_standard_deviations=1.75)

                        # Create a new message to send the resulting parameters
                        result_message = threshold_parameters()

                        # Save the individual parameters into the message
                        result_message.lower_bound = result_parameters[0]
                        result_message.upper_bound = result_parameters[1]

                        # Publish the result of the threshold generation
                        self.threshold_publisher.publish(result_message)

                else:
                    raise Exception("Action type was not recognized.")

    def raw_image_callback(self, message: image_data_message):

        # Save the latest raw image
        self.image_to_crop = ImageData(image_data_msg=message, image_color=COLOR_GRAY)

    def cropped_image_callback(self, message: image_data_message):

        # Save the latest cropped image
        self.image_for_mask_and_threshold = ImageData(image_data_msg=message, image_color=COLOR_GRAY)

    def generate_crop_coordinates_callback(self, data: Bool):

        self.actions.append(GENERATE_CROP)

    def generate_grabcut_initialization_mask_callback(self, data: Bool):

        self.actions.append(GENERATE_INITIALIZATION)

    def generate_threshold_parameters_callback(self, data: Bool):

        self.actions.append(GENERATE_PARAMETERS)


if __name__ == '__main__':
    # Create a new object instance for converting the images
    node = ImageBasedUserInput()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # Let the program run indefinitely
    node.main_loop()

