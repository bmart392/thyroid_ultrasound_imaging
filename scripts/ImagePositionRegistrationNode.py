#!/usr/bin/env python3

"""
File containing ImagePositionRegistrationNode class definition and ROS running code.
"""

# Import standard ROS packages
from rospy import init_node, Subscriber, Publisher, is_shutdown, Time
from geometry_msgs.msg import WrenchStamped
from franka_msgs.msg import FrankaState

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import image_data_message, transformed_points, Float64MultiArrayStamped

# Import standard packages
from numpy import array
from copy import copy

# Import custom packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageData.bridge_list_of_points_multi_array import \
    bridge_list_of_points_multi_array, FLOAT_ARRAY, THREE_D, FOUR_D
from thyroid_ultrasound_imaging_support.ImageData.BridgeImageDataMessageConstants import TO_MESSAGE
from thyroid_ultrasound_support.TopicNames import *


# Define constants for the indices of the object and the message
OBJECT: int = int(0)
MSG: int = int(1)


class ImagePositionRegistrationNode:
    def __init__(self):
        # Create the node object
        init_node("ImagePositionRegistrationNode")

        # Listens to /image_data/filtered to get the contour of the thyroid
        Subscriber('/image_data/filtered', image_data_message, self.filtered_image_callback)

        # Listens to the pose of the end effector
        Subscriber(ROBOT_POSE, Float64MultiArrayStamped, self.pose_callback)

        # Listens to the force at the end effector
        Subscriber(ROBOT_FORCE, WrenchStamped, self.force_callback)

        # Publishes the resulting transformed data points from registered pairs of data
        self.transformed_points_publisher = Publisher('/image_data/transformed_points', transformed_points,
                                                      queue_size=1)

        # Define a variable to store the most recent filtered images
        self.list_of_filtered_images = {}

        # Define a variable to store the most recent robot poses
        self.list_of_robot_poses = {}

        # Define a variable to store the most recent robot force messages
        self.list_of_robot_forces = {}

        # Define the image height
        self.image_height = 480  # pixels

        # Define the imaging depth of the scanner
        self.imaging_depth = 60  # mm

        # Define the oldest any piece of data is allowed to be
        self.data_age_limit = 10  # seconds

    def filtered_image_callback(self, data: image_data_message):

        # Convert the incoming image_data_message to an image_data_object
        image_data_object = ImageData(image_data_msg=data)

        # Create the key values with which it will be stored
        outer_level_key = data.header.stamp.secs
        key_value_pair = {data.header.stamp.nsecs: (image_data_object, image_data_message)}

        # Try adding the value pair to an existing dictionary
        try:
            self.list_of_filtered_images[outer_level_key].update(key_value_pair)

        # If an appropriate dictionary does not exist, create on
        except KeyError:
            self.list_of_filtered_images.update({outer_level_key: key_value_pair})

        # Remove any bad data
        self.list_of_filtered_images = self.remove_old_or_empty_data(self.list_of_filtered_images)

    def pose_callback(self, pose_msg: Float64MultiArrayStamped):

        # Define the outer level key to store in the dictionary
        outer_level_key = pose_msg.header.stamp.secs

        # Define the lower level key and the value to store in the dictionary
        key_value_pair = {pose_msg.header.stamp.nsecs: (array(pose_msg.data.data).reshape((4, 4)), pose_msg)}

        try:
            # Try to add the new data into the existing place in the dictionary
            self.list_of_robot_poses[outer_level_key].update(key_value_pair)

        except KeyError:

            # If the correct place in the dictionary does not exist yet, create it and then add the value
            self.list_of_robot_poses.update({outer_level_key: key_value_pair})

        # Remove any bad data
        self.list_of_robot_poses = self.remove_old_or_empty_data(self.list_of_robot_poses)

    def force_callback(self, force_msg: WrenchStamped):

        # Define the outer level key to store in the dictionary
        outer_level_key = force_msg.header.stamp.secs

        # Define the lower level key and the value to store in the dictionary
        key_value_pair = {force_msg.header.stamp.nsecs: (None, force_msg)}

        try:
            # Try to add the new data into the existing place in the dictionary
            self.list_of_robot_forces[outer_level_key].update(key_value_pair)

        except KeyError:

            # If the correct place in the dictionary does not exist yet, create it and then add the value
            self.list_of_robot_forces.update({outer_level_key: key_value_pair})

        # Remove any bad data
        self.list_of_robot_forces = self.remove_old_or_empty_data(self.list_of_robot_forces)

    def pixel_2_mm(self, pixel_value):
        return pixel_value * self.imaging_depth / self.image_height

    def publish_registered_data(self, image_data: ImageData, image_data_msg: image_data_message,
                                pose: array, pose_msg,
                                force_msg: WrenchStamped):
        # TODO Make a custom message that is a combination of the three message types included and then publish it here

        # Define a list to store the points after they have been transformed into the real world
        transformed_points_list = []

        # For each contour in the image
        for contour in image_data.contours_in_image:

            # For each point in the given contour
            for point in contour:
                # Transform the point into real world coordinates
                transformed_point = pose @ array([
                    array([0]),
                    array([self.pixel_2_mm(point[0])]),
                    array([self.pixel_2_mm(point[1])]),
                    array([1]),
                ])

                # Remove the extra element
                transformed_point = transformed_point[0:3].reshape(3)

                # Add the point to the list of transformed points
                transformed_points_list.append(transformed_point)

        # Change the order of the list
        transformed_points_list = array(transformed_points_list)

        # Create a new transformed_points_message
        transformed_points_msg = transformed_points()

        # Add time stamp to message
        transformed_points_msg.header.stamp = Time.now()

        # TODO THISSSSS
        # Publish the robot transformation, raw image, segmented image, robot sensed force to a node to be saved to files

        # Build a message that contains the transformed points and robot pose
        transformed_points_msg.transformed_points = \
            bridge_list_of_points_multi_array(TO_MESSAGE, list_of_points=transformed_points_list,
                                              msg_type=FLOAT_ARRAY, point_dim=THREE_D)
        transformed_points_msg.robot_pose = bridge_list_of_points_multi_array(TO_MESSAGE, list_of_points=pose,
                                                                              msg_type=FLOAT_ARRAY, point_dim=FOUR_D)

        # Publish the message
        self.transformed_points_publisher.publish(transformed_points_msg)

    def main_loop(self):

        # As long as the node is not shut down
        while not is_shutdown():

            # Copy both lists locally to avoid weird threading things
            local_list_of_filtered_images = copy(self.list_of_filtered_images)
            local_list_of_robot_poses = copy(self.list_of_robot_poses)
            local_list_of_robot_forces = copy(self.list_of_robot_forces)

            # And images are available
            if len(local_list_of_filtered_images) > 0 and len(local_list_of_robot_poses) > 0 and \
                    len(local_list_of_robot_forces) > 0:

                # Find the outer level key of the oldest image
                newest_image_key_seconds = max(local_list_of_filtered_images.keys())

                # Check to see if a robot pose was taken at the same time as the newest image
                if newest_image_key_seconds in local_list_of_robot_poses.keys() and \
                        newest_image_key_seconds in local_list_of_robot_forces.keys() and \
                        len(local_list_of_filtered_images[newest_image_key_seconds].keys()) > 0:

                    # For each nanosecond image key,
                    for image_key_nanoseconds in local_list_of_filtered_images.get(newest_image_key_seconds).keys():

                        # Create a variable to store the nanosecond key of the pose closest to the image
                        """closest_pose_nanoseconds = 0

                        # Create a flag denoting that a match has been found
                        pose_found = False"""

                        pose_found, closest_pose_nanoseconds = self.find_closest_key_value(newest_image_key_seconds,
                                                                                           image_key_nanoseconds,
                                                                                           local_list_of_robot_poses)

                        force_found, closest_force_nanoseconds = self.find_closest_key_value(newest_image_key_seconds,
                                                                                             image_key_nanoseconds,
                                                                                             local_list_of_robot_forces)

                        """# Create a variable to store the nanosecond key of the force closest to the image
                        closest_force_nanoseconds = 0
                        
                        # Create a flag denoting that a match has been found
                        force_found = False

                        # For each nanosecond pose key,
                        for pose_key_nanoseconds in local_list_of_robot_poses.get(newest_image_key_seconds).keys():

                            # Check to see if it is the closest pose to the image
                            if abs(image_key_nanoseconds - pose_key_nanoseconds) < \
                                    abs(image_key_nanoseconds - closest_pose_nanoseconds):

                                # If so, save the pose
                                closest_pose_nanoseconds = pose_key_nanoseconds

                                # Note that a pose has been found
                                pose_found = True
                                
                        # For each nanosecond force key,
                        for force_key_nanoseconds in local_list_of_robot_forces.get(newest_image_key_seconds).keys():
                            
                            # Check to see if it """

                        # If pose was found that matches the image
                        if pose_found and force_found:

                            # Publish the pose and the image
                            self.publish_registered_data(
                                local_list_of_filtered_images[newest_image_key_seconds][image_key_nanoseconds][OBJECT],
                                local_list_of_filtered_images[newest_image_key_seconds][image_key_nanoseconds][MSG],
                                local_list_of_robot_poses[newest_image_key_seconds][closest_pose_nanoseconds][OBJECT],
                                local_list_of_robot_poses[newest_image_key_seconds][closest_pose_nanoseconds][MSG],
                                local_list_of_robot_forces[newest_image_key_seconds][closest_force_nanoseconds][MSG]
                            )

                            # Remove both the pose and the image from the selection of stored data
                            self.list_of_filtered_images[newest_image_key_seconds].pop(image_key_nanoseconds)
                            self.list_of_robot_poses[newest_image_key_seconds].pop(closest_pose_nanoseconds)

                            # Break out of the looping
                            break

    def remove_old_or_empty_data(self, dictionary_to_edit: dict) -> dict:
        # Define a list to store keys that should be removed
        keys_to_pop = []

        # Get the current time
        current_time = Time.now().secs

        # Check through each dictionary to see if a dictionary is empty of the key is too old
        for outer_level_key in dictionary_to_edit.keys():
            if current_time - outer_level_key > self.data_age_limit:
                # If so, add it to the list of keys to remove
                keys_to_pop.append(outer_level_key)

        # Remove each key that has been identified
        for key_to_pop in keys_to_pop:
            dictionary_to_edit.pop(key_to_pop)

        return dictionary_to_edit

    @staticmethod
    def find_closest_key_value(seconds_key_to_search_for: int, nanoseconds_key_to_search_for: int, dict_to_search_in: dict) -> (bool, int):

        # Create a variable to store the resulting nanosecond key of the inner dict
        closest_result_nanoseconds = 0

        # Create a flag denoting that a match has been found
        result_found = False

        # For each nanosecond pose key,
        for nanoseconds_key in dict_to_search_in.get(seconds_key_to_search_for).keys():

            # Check to see if it is the closest pose to the image
            if abs(nanoseconds_key_to_search_for - nanoseconds_key) < \
                    abs(nanoseconds_key_to_search_for - closest_result_nanoseconds):
                # If so, save the pose
                closest_result_nanoseconds = nanoseconds_key

                # Note that a pose has been found
                result_found = True

        return result_found, closest_result_nanoseconds


if __name__ == '__main__':
    # create node object
    node = ImagePositionRegistrationNode()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # Run the main loop until the user exits
    node.main_loop()
