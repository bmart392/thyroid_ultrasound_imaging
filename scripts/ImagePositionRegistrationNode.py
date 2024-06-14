#!/usr/bin/env python3

"""
File containing ImagePositionRegistrationNode class definition and ROS running code.
"""
# Import standard python packages
from os import mkdir
from os.path import isdir
from copy import copy

# Import standard ROS packages
from geometry_msgs.msg import WrenchStamped
from armer_msgs.msg import ManipulatorState

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageData.BridgeImageDataMessageConstants import TO_MESSAGE
from thyroid_ultrasound_imaging_support.ImageData.bridge_list_of_points_multi_array import \
    bridge_list_of_points_multi_array, FOUR_D, FLOAT_ARRAY
from thyroid_ultrasound_support.Functions.date_stamp_str import date_stamp_str
from thyroid_ultrasound_robot_control_support.Helpers.convert_pose_to_transform_matrix import \
    convert_pose_to_transform_matrix
from thyroid_ultrasound_support.BasicNode import *

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import image_data_message, transformed_points
from thyroid_ultrasound_imaging_support.RegisteredData.RegisteredData import RegisteredData, RobotForce, RobotPose, \
    SAVE_OBJECT
from thyroid_ultrasound_services.srv import *

# Define constants for the indices of the object and the message
OBJECT: int = int(0)
MSG: int = int(1)

# Define a prefix to for each folder of stored data
FOLDER_OF_REGISTERED_DATA_PREFIX: str = '/Exam'


# TODO - Dream - Add proper try-catch error checking everywhere and incorporate logging into it
# TODO - Dream - Add logging through the BasicNode class
# TODO - Medium - Why does it register more than one data per waypoint?


class ImagePositionRegistrationNode(BasicNode):
    def __init__(self):
        """
        Creates an ImagePositionRegistrationNode object.
        """

        # Call the constructor of hte super class
        super().__init__()

        # Define a variable to store data about the most recent filtered images
        self.filtered_image = None

        # Define a variable to store the most recent robot poses
        self.list_of_robot_poses = {}

        # Define a variable to store the most recent robot force messages
        self.list_of_robot_forces = {}

        # Define the image height
        self.image_height = 480  # pixels

        # Define the imaging depth of the scanner
        self.imaging_depth = 60  # mm

        # Define the oldest any piece of data is allowed to be
        self.data_age_limit = 60  # seconds

        # Define a flag to store if the node has been commanded to register new data
        self.register_new_data_flag = False

        # Define a variable to note when an image has been saved
        self.image_saved = False

        # Define a variable to store where to save the registered data
        self.registered_data_save_location = '/home/ben/thyroid_ultrasound_data/testing_and_validation/registered_data'

        # Create the node object
        init_node(IMAGE_POSITION_REGISTRATION)

        # Define the service proxy for notifying the trajectory node that data has been registered
        self.data_has_been_registered_service = ServiceProxy(TM_DATA_HAS_BEEN_REGISTERED, BoolRequest)

        # Define the service for being requested to register data
        Service(IPR_REGISTER_NEW_DATA, BoolRequest, self.register_new_data_handler)

        # Define the service for setting the registered data save location
        Service(IPR_REGISTERED_DATA_SAVE_LOCATION, StringRequest, self.registered_data_save_location_handler)

        # Publishes the resulting transformed data points from registered pairs of data
        self.transformed_points_publisher = Publisher(IMAGE_TRANSFORMED_POINTS, transformed_points,
                                                      queue_size=1)

        # Listens to /image_data/filtered to get the contour of the thyroid
        Subscriber(IMAGE_FILTERED, image_data_message, self.filtered_image_callback)

        # Create a subscriber to listen for the robot transformation
        Subscriber(ARMER_STATE, ManipulatorState, self.robot_state_callback)

        # Listens to the force at the end effector
        Subscriber(ROBOT_DERIVED_FORCE, WrenchStamped, self.force_callback)

    ###########################
    # Data collection callbacks
    # region

    def filtered_image_callback(self, msg: image_data_message):
        """
        Given a new image data message, save it if data needs to be registered and an image has not already been saved..

        Parameters
        ----------
        msg :
            A message containing an image data object.
        """
        if self.register_new_data_flag and self.filtered_image is None:

            # Save the image data object equivalent
            self.filtered_image = ImageData(image_data_msg=msg)

    def robot_state_callback(self, msg: ManipulatorState):
        """
        Saves the given pose message in the pose history list of the node.

        Parameters
        ----------
        msg :
            A message containing the full state of the robot
        """

        # Convert the pose from the message into an array
        pose_as_matrix = convert_pose_to_transform_matrix(msg.ee_pose.pose)

        # Convert the array to a message
        pose_as_multi_array = bridge_list_of_points_multi_array(direction=TO_MESSAGE,
                                                                list_of_points=pose_as_matrix,
                                                                msg_type=FLOAT_ARRAY,
                                                                point_dim=FOUR_D)

        # Capture the current time
        current_time = Time.now()

        # Define the outer level key to store in the dictionary
        outer_level_key = current_time.secs  # msg.ee_pose.header.stamp.secs

        # Define the lower level key and the value to store in the dictionary
        key_value_pair = {current_time.nsecs: (RobotPose(robot_pose=pose_as_matrix), pose_as_multi_array)}

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
        Saves the given force message in the history list of the node.

        Parameters
        ----------
        force_msg :
            A message containing a stamped robot force.
        """

        # Define the outer level key to store in the dictionary
        outer_level_key = force_msg.header.stamp.secs

        # Define the lower level key and the value to store in the dictionary
        key_value_pair = {force_msg.header.stamp.nsecs: (RobotForce(source_message=force_msg), force_msg)}

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

    def registered_data_save_location_handler(self, req: StringRequestRequest):
        """
        Updates the internal registered_data_save_location field with the path contained in the message.
        The function will raise an exception if the path given is invalid.
        """
        if isdir(req.value):
            temp_location = req.value + FOLDER_OF_REGISTERED_DATA_PREFIX + date_stamp_str(prefix='_', suffix='')
            mkdir(temp_location)
            self.registered_data_save_location = temp_location
            return StringRequestResponse(was_succesful=True, message=NO_ERROR)
        else:
            return StringRequestResponse(was_succesful=False, message='Invalid directory.')

    def register_new_data_handler(self, req: BoolRequestRequest):
        self.register_new_data_flag = req.value
        return BoolRequestResponse(True, NO_ERROR)

    # endregion
    ##################

    ##################
    # Helper functions
    # region

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

            # Update the result to allow for comparison
            closest_result_nanoseconds = 0

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
        Removes any dictionary that is too old based on the age of the newest data.

        Parameters
        ----------
        dictionary_to_edit :
            A dictionary containing entries formatted as {seconds_key : data, ...}
        """
        # Define a list to store keys that should be removed
        keys_to_pop = []

        # Get the maximum time (in seconds) contained in the outer level dictionary
        max_time = max(dictionary_to_edit.keys())

        # Check through each dictionary to see if a dictionary is too old
        for outer_level_key in dictionary_to_edit.keys():
            if max_time - outer_level_key > self.data_age_limit:
                # If so, add it to the list of keys to remove
                keys_to_pop.append(outer_level_key)

        # Remove each key that has been identified
        for key_to_pop in keys_to_pop:
            dictionary_to_edit.pop(key_to_pop)

        return dictionary_to_edit

    # endregion
    ##################

    def main_loop(self):

        if self.register_new_data_flag:

            # Wait until a new filtered image has been received
            if self.filtered_image is None:
                return

            # Create a local copy of the filtered image
            local_filtered_image = copy(self.filtered_image)

            # Find the outer level key of the oldest image
            newest_image_seconds = local_filtered_image.image_capture_time.secs

            # Wait until a robot pose was taken at the same time as the newest image
            if newest_image_seconds not in self.list_of_robot_poses.keys():
                return

            # Wait until a robot force was taken at the same time as the newest image
            if newest_image_seconds not in self.list_of_robot_forces.keys():
                return

            # Copy all data locally to avoid weird threading things
            local_list_of_robot_poses = copy(self.list_of_robot_poses[newest_image_seconds])
            local_list_of_robot_forces = copy(self.list_of_robot_forces[newest_image_seconds])

            # Pull out the nanoseconds stamp of the most recently filtered image
            newest_image_nanoseconds = local_filtered_image.image_capture_time.nsecs

            # Find the closest nanoseconds keys of the pose and the force if they exist
            closest_pose_nanoseconds = self.find_closest_key_value(newest_image_seconds,
                                                                   newest_image_nanoseconds,
                                                                   {newest_image_seconds: local_list_of_robot_poses})

            closest_force_nanoseconds = self.find_closest_key_value(newest_image_seconds,
                                                                    newest_image_nanoseconds,
                                                                    {newest_image_seconds: local_list_of_robot_forces})

            # If a pose and a force were found that match the image
            if closest_pose_nanoseconds is not None and closest_force_nanoseconds is not None:
                # Publish the pose and the image
                new_registered_data = RegisteredData(image_data_object=local_filtered_image,
                                                     robot_pose=local_list_of_robot_poses[
                                                         closest_pose_nanoseconds][
                                                         OBJECT],
                                                     robot_force=local_list_of_robot_forces[
                                                         closest_force_nanoseconds][
                                                         OBJECT],
                                                     )

                # Save the new registered data object
                new_registered_data.save_load(action=SAVE_OBJECT,
                                              path_to_file_location=self.registered_data_save_location)
                # Publish the registered data
                # self.registered_data_publisher.publish(new_registered_data.convert_object_message(TO_MESSAGE))

                # Remove both the pose and the force from the selection of stored data
                try:
                    self.list_of_robot_poses[newest_image_seconds].pop(closest_pose_nanoseconds)
                except KeyError:
                    pass
                try:
                    self.list_of_robot_forces[newest_image_seconds].pop(closest_pose_nanoseconds)
                except KeyError:
                    pass

                self.data_has_been_registered_service(True)
                self.filtered_image = None
                self.register_new_data_flag = False


if __name__ == '__main__':
    # create node object
    node = ImagePositionRegistrationNode()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    while not is_shutdown():
        node.main_loop()

    print("Node terminated.")
