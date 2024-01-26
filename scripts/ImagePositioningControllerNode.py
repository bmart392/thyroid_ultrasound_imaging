#!/usr/bin/env python3

"""
File containing ImagePositioningControllerNode class definition and ROS running code.
"""

# Import standard ROS packages
from rospy import init_node, Subscriber, Publisher, is_shutdown, Rate
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Float64

# Import standard packages
from copy import copy

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import image_data_message

# Import custom packages
from thyroid_ultrasound_imaging_support.Controller.ImagePositioningController import ImagePositioningController
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData


"""
This node should read the location of the centroid off of the filtered image message and then do its work from there.
"""



"""
# If the thyroid is present in the image, publish data about it
        if thyroid_in_image:

            

            # Analyze the centroid location to determine the needed control input, current thyroid position error, and
            # if the thyroid is in the center of the image
            control_input_msg, current_error, is_thyroid_centered = self.image_positioning_controller. \
                calculate_control_input(self.image_data)

            # note the time required to calculate the image based control input
            start_of_process_time = display_process_timer(start_of_process_time,
                                                          "Image based control input calculation",
                                                          self.analysis_mode)

            # Publish if the thyroid is centered
            self.is_thyroid_centered_status_publisher.publish(Bool(is_thyroid_centered))

            # publish the image based control input
            self.image_based_control_input_publisher.publish(control_input_msg)

        # Publish no control input and do not show the thyroid as centered
        else:
            self.is_thyroid_centered_status_publisher.publish(Bool(False))
            self.image_based_control_input_publisher.publish(TwistStamped())
            """


# TODO - Medium - Change topic names to use constants from topics files
class ImagePositioningControllerNode:

    def __init__(self):

        # Create the node object
        init_node('ImagePositioningControllerNode')

        # Define a subscriber to listen for the imaging depth of the scanner
        Subscriber('/image_data/imaging_depth', Float64, self.imaging_depth_callback)

        # Define a subscriber to listen for the filtered images
        Subscriber('/image_data/filtered', image_data_message, self.filtered_image_callback)

        # Define a publisher to publish the image-based positioning error
        self.image_based_position_error_publisher = Publisher(
            '/image_control/distance_to_centroid', TwistStamped, queue_size=1
        )

        # Define a publisher to publish if the image is centered
        self.image_centered_publisher = Publisher('status/image_centered', Bool, queue_size=1)

        # Define a variable to store the newest image data available
        # noinspection PyTypeChecker
        self.newest_image: ImageData = None

        # Define a variable to store the position error based on the newest image
        self.position_error = None

        # Define a positioning controller object to use in the object
        self.image_positioning_controller = ImagePositioningController()

        # Define a variable to save the imaging depth of the scanner
        self.image_positioning_controller.imaging_depth = 5.0

    def imaging_depth_callback(self, data: Float64):

        # Save the newest imaging depth
        self.image_positioning_controller.imaging_depth = data.data
        print(data.data)

    def filtered_image_callback(self, data: image_data_message):

        # Save the newest filtered image
        self.newest_image = ImageData(image_data_msg=data)

    def publish_position_error(self):

        # Check that there is an image to use
        if self.newest_image is not None:

            # Create a copy of the newest image data to work with
            image_data = copy(self.newest_image)

            # Calculate the error in the image centroids
            position_error, is_image_centered = self.image_positioning_controller.calculate_position_error(image_data)

            # Publish if the image is centered
            self.image_centered_publisher.publish(Bool(is_image_centered))

            # Create a new TwistStamped message
            position_error_message = TwistStamped()

            # Fill in the message fields converting from the image axes to the robot end effector axes
            position_error_message.twist.linear.x = position_error[0]
            position_error_message.twist.angular.y = position_error[5]

            # Publish positioning error
            self.image_based_position_error_publisher.publish(position_error_message)


if __name__ == '__main__':

    # create node object
    controller = ImagePositioningControllerNode()

    # Define publishing frequency
    publishing_rate = Rate(100)  # hz

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # TODO - Medium - Add logic to stop having it send messages when a new message has not arrived
    # Let the program run indefinitely
    while not is_shutdown():

        # Publish the image based control input
        controller.publish_position_error()

        # Then wait
        publishing_rate.sleep()





