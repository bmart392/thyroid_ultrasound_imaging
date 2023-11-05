#!/usr/bin/env python3

"""
File containing ImagePositionRegistrationNode class definition and ROS running code.
"""

# Import standard ROS packages
from rospy import init_node, Subscriber, Publisher, is_shutdown, Time
from franka_msgs.msg import FrankaState

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import image_data_message, transformed_points, Float64MultiArrayStamped

# Import standard packages
from numpy import array
from copy import copy

# Import custom packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageData.bridge_list_of_points_multi_array import \
    bridge_list_of_points_multi_array
from thyroid_ultrasound_imaging_support.ImageData.BridgeImageDataMessageConstants import TO_MESSAGE
from thyroid_ultrasound_support.TopicNames import *


class ImagePositionRegistrationNode:
    def __init__(self):
        # Create the node object
        init_node("ImagePositionRegistrationNode")

        # Listens to /image_data/filtered to get the contour of the thyroid
        Subscriber('/image_data/filtered', image_data_message, self.filtered_image_callback)

        # TODO Fix this to work with the actual position getting sent out
        # Maybe make new message class
        # Listens to the robot to get the pose of the end effector
        Subscriber(ROBOT_POSE, Float64MultiArrayStamped, self.pose_callback)

        # Publishes the resulting transformed data points from registered pairs of data
        self.transformed_points_publisher = Publisher('/image_data/transformed_points', transformed_points,
                                                      queue_size=1)

        # Define a variable to store the most recent filtered images
        self.list_of_filtered_images = {}

        # Define a variable to store the most recent robot poses
        self.list_of_robot_poses = {}

        # Define the image height
        self.image_height = 480  # pixels

        # Define the imaging depth of the scanner
        self.imaging_depth = 60  # mm

    def filtered_image_callback(self, data: image_data_message):
        # Convert the incoming image_data_message to an image_data_object
        # then save it in the list

        # TODO add timestamp to filtered image messages
        image_data_object = ImageData(image_data_msg=data)
        outer_level_key = data.header.stamp.secs
        key_value_pair = {data.header.stamp.nsecs: image_data_object}
        try:
            self.list_of_filtered_images[outer_level_key].update(key_value_pair)

        except KeyError:
            self.list_of_filtered_images.update({outer_level_key: key_value_pair})

    def pose_callback(self, pose: Float64MultiArrayStamped):

        # Define the outer level key to store in the dictionary
        outer_level_key = pose.header.stamp.secs

        # Define the lower level key and the value to store in the dictionary
        key_value_pair = {pose.header.stamp.nsecs: array(pose.data.data).reshape((4, 4))}

        try:
            # Try to add the new data into the existing place in the dictionary
            self.list_of_robot_poses[outer_level_key].update(key_value_pair)

        except KeyError:

            # If the correct place in the dictionary does not exist yet, create it and then add the value
            self.list_of_robot_poses.update({outer_level_key: key_value_pair})

        # Save the current time in seconds
        current_time = Time.now().secs

        # Define a variable to mark how long to save data for
        oldest_allowed_data = 5  # seconds

        # Remove any data in the dictionary that is too old
        for time in self.list_of_robot_poses.keys():
            if current_time - time >= oldest_allowed_data:
                self.list_of_robot_poses.pop(time)

    def pixel_2_mm(self, pixel_value):
        return pixel_value * self.imaging_depth / self.image_height

    def publish_registered_data(self, image_data: ImageData, pose: array):

        # Define a list to store the points after they have been transformed into the real world
        transformed_points_list = []

        # For each contour in the image
        for contour in image_data.contours_in_image:

            # For each point in the given contour
            for point in contour:
                # Transform the point into real world coordinates
                transformed_points_list.append(pose * array([  # TODO Fix this transformation matrix
                    array([0]),
                    array([self.pixel_2_mm(point[0])]),
                    array([self.pixel_2_mm(point[1])]),
                    array([1]),
                ]))

        # Create a new transformed_points_message
        transformed_points_msg = transformed_points()

        # Build a message that contains the transformed points and robot pose
        transformed_points_msg.transformed_points = \
            bridge_list_of_points_multi_array(TO_MESSAGE, list_of_points=transformed_points_list)
        transformed_points_msg.robot_pose = bridge_list_of_points_multi_array(TO_MESSAGE, list_of_points=pose)

        # Publish the message
        self.transformed_points_publisher.publish(transformed_points_msg)

    def main_loop(self):

        # As long as the node is not shut down
        while not is_shutdown():

            # And images are available
            if len(self.list_of_filtered_images) > 0:

                # Find the outer level key of the oldest image
                oldest_image_seconds = min(self.list_of_filtered_images.keys())

                # Find the inner level key of the oldest image
                oldest_image_nanoseconds = min(self.list_of_filtered_images.get(
                    oldest_image_seconds).keys())

                # Save a reference to the oldest filtered image
                oldest_filtered_image: ImageData = self.list_of_filtered_images.get(
                    oldest_image_seconds).get(oldest_image_nanoseconds)

                # Save a copy of the most recent version of the list of poses
                most_recent_list_of_poses = copy(self.list_of_robot_poses)

                try:
                    # Try to find all the poses that occurred in the same second as the image
                    possible_corresponding_poses: dict = most_recent_list_of_poses.pop(oldest_image_seconds)

                    # Get a list of the keys in the possible poses
                    keys = possible_corresponding_poses.keys()

                    # Define a variable for the maximum allowed error between an image time and a pose time
                    max_allowed_error = 100000

                    for key in keys:

                        # If a corresponding key has been found
                        if abs(key - oldest_image_nanoseconds) <= max_allowed_error:
                            # Publish the registered data
                            self.publish_registered_data(oldest_filtered_image, possible_corresponding_poses[key])

                            # Remove the pose from the full dictionary of stored poses
                            self.list_of_robot_poses[oldest_image_seconds].pop(key)

                            # Remove the image data from the full list of stored images
                            self.list_of_filtered_images.pop(0)

                except KeyError:
                    pass
                    # Remove the oldest image data from the list of stored images
                    # oldest_filtered_image = self.list_of_filtered_images.pop(0)

                    # Add it to the back of the list so that other data can be registered
                    # self.list_of_filtered_images.append(oldest_filtered_image)


if __name__ == '__main__':
    # create node object
    node = ImagePositionRegistrationNode()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # Run the main loop until the user exits
    node.main_loop()
