#!/usr/bin/env python3

"""
File containing ImagePositioningControllerNode class definition and ROS running code.
"""

# TODO - Dream - Add proper try-cath error checking everywhere and incorporate logging into it
# TODO - Dream - Add proper node status publishing
# TODO - Dream - Add a check to make sure that any image processed by the controller is no less than half a second old

# Import standard ROS packages
from geometry_msgs.msg import TwistStamped

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import image_data_message

# Import custom packages
from thyroid_ultrasound_imaging_support.Controller.ImagePositioningController import ImagePositioningController
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_support.BasicNode import *


class ImagePositioningControllerNode(BasicNode):

    def __init__(self):

        # Call the init of the super class
        super().__init__()

        # Define a variable to store the received images
        self.received_images = []

        # Define the maximum number of images to save
        self.max_images_to_store = 25

        # Define a variable to store the position error based on the newest image
        self.position_error = None

        # Define a positioning controller object to use in the object
        self.image_positioning_controller = ImagePositioningController()

        # Define a variable to save the imaging depth of the scanner
        self.image_positioning_controller.imaging_depth = 5.0

        # Create the node object
        init_node(IMAGE_POSITIONING_CONTROLLER)

        # Define a publisher to publish the image-based positioning error
        self.image_based_position_error_publisher = Publisher(
            RC_IMAGE_ERROR, TwistStamped, queue_size=1
        )

        # Define a publisher to publish if the image is centered
        self.image_centered_publisher = Publisher(IMAGE_ROI_CENTERED, Bool, queue_size=1)

        # Define a subscriber to listen for the imaging depth of the scanner
        Subscriber(IMAGE_DEPTH, Float64, self.imaging_depth_callback)

        # Define a subscriber to listen for the filtered images
        Subscriber(IMAGE_FILTERED, image_data_message, self.filtered_image_callback)

    def imaging_depth_callback(self, data: Float64):

        # Save the newest imaging depth
        self.image_positioning_controller.imaging_depth = data.data

    def filtered_image_callback(self, data: image_data_message):
        """
        Updates the list of received images and places the newest message at the end of the list.
        """

        # Create new image data based on received image and add it to the list
        self.received_images.append(ImageData(image_data_msg=data))

        # Remove the oldest image if the list is now too long
        if len(self.received_images) > self.max_images_to_store:
            self.received_images.pop(0)

    def publish_position_error(self):

        # Check that there is an image to use
        if len(self.received_images) > 0:

            # Pop out the newest data
            image_data = self.received_images.pop(-1)

            try:

                # Calculate the error in the image centroids
                position_error, is_image_centered = self.image_positioning_controller.calculate_position_error(image_data)

            except Exception as caught_exception:

                # Define a zero error value as default
                position_error = [0, 0, 0, 0, 0, 0]

                # Define a default value for is_image_centered
                is_image_centered = False

            # Publish if the image is centered
            self.image_centered_publisher.publish(Bool(is_image_centered))

            # Create a new TwistStamped message
            position_error_message = TwistStamped()

            # Fill in the message fields converting from the image axes to the robot end effector axes
            position_error_message.twist.linear.x = position_error[0]
            position_error_message.twist.angular.y = position_error[5]

            # Publish positioning error
            self.image_based_position_error_publisher.publish(position_error_message)

            # Return the results for validation
            return position_error, is_image_centered


if __name__ == '__main__':

    # create node object
    controller = ImagePositioningControllerNode()

    # Define publishing frequency
    publishing_rate = Rate(100)  # hz

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # Let the program run indefinitely
    while not is_shutdown():

        # Publish the image based control input
        controller.publish_position_error()

        # Then wait
        publishing_rate.sleep()





