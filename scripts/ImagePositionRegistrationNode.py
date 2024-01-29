#!/usr/bin/env python3

"""
File containing ImagePositionRegistrationNode class definition and ROS running code.
"""

# Import standard ROS packages
from rospy import Time
from geometry_msgs.msg import WrenchStamped

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import image_data_message, transformed_points, Float64MultiArrayStamped, \
    RegisteredData

# Import standard packages
from numpy import array, median
from copy import copy
from cv2 import contourArea

# Import custom packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageData.bridge_list_of_points_multi_array import \
    bridge_list_of_points_multi_array, FLOAT_ARRAY, THREE_D, FOUR_D
from thyroid_ultrasound_imaging_support.ImageData.BridgeImageDataMessageConstants import TO_MESSAGE
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_support.TopicNames import *

# Define constants for the indices of the object and the message
OBJECT: int = int(0)
MSG: int = int(1)

# TODO - Dream - Add proper try-cath error checking everywhere and incorporate logging into it
# TODO - Dream - Add node status publishing
# TODO - High - Create a method of saving registered data sets to my computer


class ImagePositionRegistrationNode(BasicNode):
    def __init__(self):

        # Call the constructor of hte super class
        super().__init__()

        # Create the node object
        init_node("ImagePositionRegistrationNode")

        # Listens to /image_data/filtered to get the contour of the thyroid
        Subscriber(IMAGE_FILTERED, image_data_message, self.filtered_image_callback)

        # Listens to the pose of the end effector
        Subscriber(ROBOT_DERIVED_POSE, Float64MultiArrayStamped, self.pose_callback)

        # Listens to the force at the end effector
        Subscriber(ROBOT_DERIVED_FORCE, WrenchStamped, self.force_callback)

        # Listens for the if the current pose goal has been reached
        Subscriber(RC_POSITION_GOAL_REACHED, Bool, self.pose_goal_callback)

        # Listens for the command to use pose control
        Subscriber(USE_POSE_FEEDBACK, Bool, self.pose_command_callback)

        # Publishes the resulting transformed data points from registered pairs of data
        self.transformed_points_publisher = Publisher(IMAGE_TRANSFORMED_POINTS, transformed_points,
                                                      queue_size=1)

        # Publishes the registered data sets generated by the node
        self.registered_data_publisher = Publisher(REGISTERED_DATA, RegisteredData, queue_size=1)

        # Define a variable to store the most recent filtered images
        self.list_of_filtered_images = {}

        # Define a variable to store the area of the largest contour over the last number of images
        self.history_of_contour_area = []
        self.times_of_contour_areas = []
        self.max_contour_area_age = 30  # seconds
        self.max_length_of_contour_history = 10
        self.min_length_of_contour_history = 3
        self.image_delay_goal = 3
        self.current_image_delay = 0

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

        # Define a variable to store if the position goal has been reached
        self.pose_goal_reached = False

        # Define a variable to store if the command to use pose feedback has been used
        self.use_pose_feedback_command = False

    ###########################
    # Data collection callbacks
    # region

    def filtered_image_callback(self, data: image_data_message):
        """
        Given a new image data message, add it to the list of data to register depending on whether pose feedback is
        in use and the area of the foreground is consistent with previously received data.
        """

        # Convert the incoming image_data_message to an image_data_object
        image_data_object = ImageData(image_data_msg=data)

        # Only add data if pose_feedback is being used and the goal has been reached OR
        # pose feedback is not being used
        if (self.use_pose_feedback_command and self.pose_goal_reached) or not self.use_pose_feedback_command:

            # Calculate the area of the largest contour in the new image data
            new_image_contour_area = contourArea(image_data_object.contours_in_image[0])

            # Check to make sure enough data history has been stored
            if len(self.history_of_contour_area) >= self.min_length_of_contour_history:

                # Temporarily create an array of the history of the area
                temp_array = array(self.history_of_contour_area)

                # Define a variable to note that the area of the contour has stabilized
                has_area_stabilized = True

                # Define a list of areas to specifically compare with the new image area
                areas_to_compare_against = [median(temp_array), self.history_of_contour_area[-1],
                                            self.history_of_contour_area[-2],
                                            self.history_of_contour_area[-3]]

                # For each area to compare against
                for area in areas_to_compare_against:

                    # If the new image contour area is greater than 5% different from the comparison area
                    if new_image_contour_area > (area + (0.05 * area)) or \
                            new_image_contour_area < (area - (0.05 * area)):

                        # Note that the area has not stabilized
                        has_area_stabilized = False

                        # Break out of the loop
                        break

                # If the area has stabilized,
                if has_area_stabilized:
                    # Create the key values with which it will be stored
                    outer_level_key = data.header.stamp.secs
                    key_value_pair = {data.header.stamp.nsecs: (image_data_object, image_data_message)}

                    # Try adding the value pair to an existing dictionary
                    try:
                        self.list_of_filtered_images[outer_level_key].update(key_value_pair)

                    # If an appropriate dictionary does not exist, create one
                    except KeyError:
                        self.list_of_filtered_images.update({outer_level_key: key_value_pair})

                    # Remove any bad data
                    self.list_of_filtered_images = self.remove_old_data(self.list_of_filtered_images)

            # Note the current time
            now_stamp = Time.now()
            now = now_stamp.secs + (now_stamp.nsecs / 10 ** 9)

            # Define a variable to store the index of where the data stops being too old
            keep_index = 0

            # Go through each time index until data that is not too old is found
            for time in self.times_of_contour_areas:
                if time - now >= self.data_age_limit:
                    break
                keep_index = keep_index + 1

            # Remove any old data
            self.history_of_contour_area = self.history_of_contour_area[keep_index::]
            self.times_of_contour_areas = self.times_of_contour_areas[keep_index::]

            # Trim the lists if too much data is stored
            self.history_of_contour_area = self.history_of_contour_area[-self.max_length_of_contour_history:]
            self.times_of_contour_areas = self.times_of_contour_areas[-self.max_length_of_contour_history:]

            # Add the contour area of the new image to the contour history
            self.history_of_contour_area.append(new_image_contour_area)

            # Add the time the contour was taken to the contour history
            self.times_of_contour_areas.append((image_data_object.image_capture_time.secs +
                                                (image_data_object.image_capture_time.nsecs / 10 ** 9))
                                               )

    def pose_callback(self, pose_msg: Float64MultiArrayStamped):
        """
        Save the given pose message in the pose history list of the node.
        """

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
        self.list_of_robot_poses = self.remove_old_data(self.list_of_robot_poses)

    def force_callback(self, force_msg: WrenchStamped):
        """
        Save the given force in the history list of the node.
        """

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
        self.list_of_robot_forces = self.remove_old_data(self.list_of_robot_forces)

    # endregion
    ###########################

    ##################
    # Status callbacks
    # region
    def pose_goal_callback(self, data: Bool):
        self.pose_goal_reached = data.data

    def pose_command_callback(self, data: Bool):
        self.use_pose_feedback_command = data.data

    # endregion
    ##################

    ##################
    # Helper functions
    # region
    def pixel_2_mm(self, pixel_value):
        return pixel_value * self.imaging_depth / self.image_height

    @staticmethod
    def find_closest_key_value(seconds_key_to_search_for: int, nanoseconds_key_to_search_for: int,
                               dict_to_search_in: dict) -> int:
        """
        Returns the data point contained in a {second_key : {nano_second_key : data, ...}, ...} if the given second
        and nanosecond keys are contained in the given dictionary.

        Parameters
        ----------
        seconds_key_to_search_for
            The outer level key to search for in the dictionary.
        nanoseconds_key_to_search_for
            The inner level key to search for in the lower level dictionary.
        dict_to_search_in
            The two-tier dictionary in which to search.
        """

        # Create a variable to store the resulting nanosecond key of the inner dict
        closest_result_nanoseconds = None

        # If a lower level dictionary exists with the given second key,
        if dict_to_search_in.get(seconds_key_to_search_for) is not None:

            # For each nanosecond pose key in that lower level dictionary,
            for nanoseconds_key in dict_to_search_in.get(seconds_key_to_search_for).keys():

                # Check to see if it is the closest pose to the image
                if abs(nanoseconds_key_to_search_for - nanoseconds_key) < \
                        abs(nanoseconds_key_to_search_for - closest_result_nanoseconds):
                    # If so, save the pose
                    closest_result_nanoseconds = nanoseconds_key

        return closest_result_nanoseconds

    def remove_old_data(self, dictionary_to_edit: dict) -> dict:
        """
        Removes any dictionary that is too old.
        """
        # Define a list to store keys that should be removed
        keys_to_pop = []

        # Get the current time
        current_time = Time.now().secs

        # Check through each dictionary to see if a dictionary is too old
        for outer_level_key in dictionary_to_edit.keys():
            if current_time - outer_level_key > self.data_age_limit:
                # If so, add it to the list of keys to remove
                keys_to_pop.append(outer_level_key)

        # Remove each key that has been identified
        for key_to_pop in keys_to_pop:
            dictionary_to_edit.pop(key_to_pop)

        return dictionary_to_edit

    # endregion
    ##################

    def publish_registered_data(self, image_data: ImageData, image_data_msg: image_data_message,
                                pose: array, pose_msg,
                                force_msg: WrenchStamped):

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

        # Generate a new message containing the image data, robot sensed force, and robot pose
        registered_data_msg = RegisteredData()
        registered_data_msg.header.stamp = Time.now()
        registered_data_msg.image_data = image_data_msg
        registered_data_msg.force = force_msg
        registered_data_msg.pose = pose_msg

        # Publish the message
        self.registered_data_publisher.publish(registered_data_msg)

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

                        # Find the closest nanoseconds keys of the pose and the force if they exist
                        closest_pose_nanoseconds = self.find_closest_key_value(newest_image_key_seconds,
                                                                               image_key_nanoseconds,
                                                                               local_list_of_robot_poses)

                        closest_force_nanoseconds = self.find_closest_key_value(newest_image_key_seconds,
                                                                                image_key_nanoseconds,
                                                                                local_list_of_robot_forces)

                        # If a pose and a force were found that match the image
                        if closest_pose_nanoseconds is not None and closest_force_nanoseconds is not None:
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


if __name__ == '__main__':
    # create node object
    node = ImagePositionRegistrationNode()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # Run the main loop until the user exits
    node.main_loop()
