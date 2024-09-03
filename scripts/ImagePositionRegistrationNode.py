#!/usr/bin/env python3

"""
File containing ImagePositionRegistrationNode class definition and ROS running code.
"""
# Import standard python packages
from os import mkdir
from os.path import isdir
from copy import copy, deepcopy
from cv_bridge import CvBridge
from numpy import array, median, ceil, reshape

# Import standard ROS packages
from geometry_msgs.msg import WrenchStamped
from armer_msgs.msg import ManipulatorState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

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


# TODO - Medium - Why does it register more than one data per waypoint?


class ImagePositionRegistrationNode(BasicNode):
    def __init__(self):
        """
        Creates an ImagePositionRegistrationNode object.
        """

        # Call the constructor of hte super class
        super().__init__()

        # Define a variable to store data about the most recent filtered image to use for registering data
        self.filtered_image_for_registering_data = None

        # Define a variable to store data about the most recent filtered image to use for saving valid positions
        self.filtered_image_for_saving_valid_positions = None

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

        # Define a flag to know when to save a valid position
        self.save_valid_positions = False

        # Define two lists to store the last valid positions and masks
        self.last_valid_positions = []
        self.last_valid_masks = []
        self.max_num_valid_data_points = 10

        # Define the length of time between saving valid data
        self.valid_position_time_interval = 0.25

        # Define variables to use to calculate the necessary time offset for data coming from the robot
        self.time_of_first_pose_message = None
        self.measurement_delay_time = Duration(secs=3)  # seconds
        self.number_of_messages_to_count = 500
        self.measured_delays = []
        self.calculated_time_difference = Duration()
        self.has_time_delay_been_reached = False
        self.has_time_difference_been_calculated = False

        # Create the node object
        init_node(IMAGE_POSITION_REGISTRATION)

        # Define a variable to store the last time a valid position and mask was saved
        self.time_of_last_valid_position = Time.now()

        # Define the service proxy for notifying the trajectory node that data has been registered
        self.data_has_been_registered_service = ServiceProxy(TM_DATA_HAS_BEEN_REGISTERED, BoolRequest)

        # Define the service for being requested to register data
        Service(IPR_REGISTER_NEW_DATA, BoolRequest, self.register_new_data_handler)

        # Define the service for setting the registered data save location
        Service(IPR_REGISTERED_DATA_SAVE_LOCATION, StringRequest, self.registered_data_save_location_handler)

        # Define the service for starting to save valid data
        Service(IPR_SAVE_VALID_POSITIONS, SaveValidPositionsSettings, self.save_valid_positions_handler)

        # Define the service for retrieving valid data
        Service(IPR_RETRIEVE_VALID_DATA, ValidData, self.retrieve_valid_data_handler)

        # Publishes the resulting transformed data points from registered pairs of data
        self.transformed_points_publisher = Publisher(IMAGE_TRANSFORMED_POINTS, transformed_points,
                                                      queue_size=1)

        # Listens to /image_data/filtered to get the contour of the thyroid
        Subscriber(IMAGE_FILTERED, image_data_message, self.filtered_image_callback)

        # Create a subscriber to listen for the robot transformation
        Subscriber(ARMER_STATE, ManipulatorState, self.robot_state_callback)

        # Listens to the force at the end effector
        Subscriber(ROBOT_DERIVED_FORCE, WrenchStamped, self.force_callback)

        # Save the current time as the last time an image was published
        self.time_of_last_publishing = Time.now()

    ###########################
    # Data collection callbacks
    # region

    def filtered_image_callback(self, msg: image_data_message):
        """
        Given a new image data message, save it if data needs to be registered and an image has not already been saved
        or if valid positions are being saved.

        Parameters
        ----------
        msg :
            A message containing an image data object.
        """
        if self.register_new_data_flag and self.filtered_image_for_registering_data is None:
            self.filtered_image_for_registering_data = ImageData(image_data_msg=msg)
        if self.save_valid_positions and self.filtered_image_for_saving_valid_positions is None:
            self.filtered_image_for_saving_valid_positions = ImageData(image_data_msg=msg)

    def robot_state_callback(self, msg: ManipulatorState):
        """
        Saves the given pose message in the pose history list of the node.

        Parameters
        ----------
        msg :
            A message containing the full state of the robot
        """

        # If the time difference between the robot pose message time stamps and
        # the current time at this node not has been calculated
        if not self.has_time_difference_been_calculated:

            # Note the time that this message arrived if it is the first message to arrive
            if self.time_of_first_pose_message is None:
                self.time_of_first_pose_message = msg.ee_pose.header.stamp

            # Otherwise,
            else:
                # Get the current time at this node
                current_time = Time.now()

                # If the node has already waited the required amount of time before calculating the difference
                if self.has_time_delay_been_reached:

                    # Record the difference between the current time and the timestamp
                    self.measured_delays.append((current_time - msg.ee_pose.header.stamp).to_sec())

                    # If enough data has been recorded,
                    if len(self.measured_delays) >= self.number_of_messages_to_count:
                        # Calculate the median delay time as a duration
                        self.calculated_time_difference = Duration().from_sec(median(array(self.measured_delays)))
                        self.has_time_difference_been_calculated = True

                # Otherwise, if the node has delayed for long enough, note that data can be collected now
                elif current_time - self.time_of_first_pose_message > self.measurement_delay_time:
                    self.has_time_delay_been_reached = True

        # Convert the pose from the message into an array
        pose_as_matrix = convert_pose_to_transform_matrix(msg.ee_pose.pose)

        # Convert the array to a message
        pose_as_multi_array = bridge_list_of_points_multi_array(direction=TO_MESSAGE,
                                                                list_of_points=pose_as_matrix,
                                                                msg_type=FLOAT_ARRAY,
                                                                point_dim=FOUR_D)

        # Capture the time of the message
        message_time = msg.ee_pose.header.stamp - self.calculated_time_difference

        # Define the outer level key to store in the dictionary
        outer_level_key = message_time.secs  # msg.ee_pose.header.stamp.secs

        # Define the lower level key and the value to store in the dictionary
        key_value_pair = {message_time.nsecs: (RobotPose(robot_pose=pose_as_matrix), pose_as_multi_array)}

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
            self.log_single_message('New save location for registered data set to: ' + str(temp_location))
            return StringRequestResponse(was_successful=True, message=NO_ERROR)
        else:
            return StringRequestResponse(was_successful=False, message='Invalid directory.')

    def save_valid_positions_handler(self, req: SaveValidPositionsSettingsRequest):
        if req.time_between_saved_poses_secs == 0 and req.length_of_history_secs == 0:
            self.save_valid_positions = False
            self.log_single_message('Valid positions no longer saved')
        elif 0 < req.time_between_saved_poses_secs < req.length_of_history_secs:
            self.save_valid_positions = True
            self.valid_position_time_interval = Duration(secs=req.time_between_saved_poses_secs)
            self.max_num_valid_data_points = ceil(req.length_of_history_secs / req.time_between_saved_poses_secs)
            self.log_single_message('Valid positions saved every ' + str(round(req.time_between_saved_poses_secs, 3)) +
                                    ' seconds covering a history of ' +
                                    str(self.max_num_valid_data_points * self.valid_position_time_interval.to_sec()) +
                                    ' seconds')
        else:
            return SaveValidPositionsSettingsResponse(False, 'Invalid settings were given')
        return SaveValidPositionsSettingsResponse(True, NO_ERROR)

    def register_new_data_handler(self, req: BoolRequestRequest):
        if req.value:
            self.log_single_message('Registration of new data requested')
        else:
            self.log_single_message('Registration of new data cancelled.')
        self.register_new_data_flag = req.value
        return BoolRequestResponse(True, NO_ERROR)

    # endregion
    ##################

    ##################
    # Service callbacks
    # region

    def retrieve_valid_data_handler(self, req: ValidDataRequest):

        # Log that data was requested
        self.log_single_message('Valid data at index ' + str(req.data_point_index) + ' to be retrieved')

        # Make copies of the data that will be sent
        try:
            pose_to_send = deepcopy(self.last_valid_positions[req.data_point_index].robot_pose)
        except IndexError:
            return ValidDataResponse(valid_pose=None, valid_mask=None, was_successful=False,
                                     message='Requested index of ' + str(req.data_point_index) +
                                             ' does not exist for robot pose')
        try:
            mask_to_send = deepcopy(self.last_valid_masks[req.data_point_index])
        except IndexError:
            return ValidDataResponse(valid_pose=None, valid_mask=None, was_successful=False,
                                     message='Requested index of ' + str(req.data_point_index) +
                                             ' does not exist for saved masks')

        # Create the message that will contain the pose
        pose_message = Float64MultiArray()

        # Add the data to the message
        pose_message.data = reshape(pose_to_send, (pose_to_send.shape[0] * pose_to_send.shape[1]))

        # Define the data padding to be zero
        pose_message.layout.data_offset = 0

        # Define the layout of the array
        pose_message.layout.dim.append(MultiArrayDimension())
        pose_message.layout.dim[0].label = "rows"
        pose_message.layout.dim[0].size = pose_to_send.shape[0]
        pose_message.layout.dim[0].stride = pose_to_send.shape[0] * pose_to_send.shape[1]

        pose_message.layout.dim.append(MultiArrayDimension())
        pose_message.layout.dim[1].label = "columns"
        pose_message.layout.dim[1].size = pose_to_send.shape[1]
        pose_message.layout.dim[1].stride = 0

        # Create a bridge to convert the mask
        bridge = CvBridge()

        # Convert the mask to an image
        mask_message = bridge.cv2_to_imgmsg(mask_to_send)

        return ValidDataResponse(valid_pose=pose_message,
                                 valid_mask=mask_message,
                                 was_successful=True, message=NO_ERROR)

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

        # Set the default status
        new_status = None
        default_status = WAITING

        # Create a local variable to hold the filtered image that will be used for this iteration
        filtered_image_to_use = None

        # If registering data,
        if self.register_new_data_flag:
            filtered_image_to_use = self.filtered_image_for_registering_data

            # Update the status
            default_status = WAITING_FOR_FILTERED_IMAGE

        # Otherwise, if saving a valid position
        elif (self.save_valid_positions and
              Time.now() - self.time_of_last_valid_position > self.valid_position_time_interval):
            filtered_image_to_use = self.filtered_image_for_saving_valid_positions

            # Update the status
            default_status = WAITING_FOR_FILTERED_IMAGE_FOR_SAVING_VALID_POSITION

        # Wait until a new filtered image has been received
        if filtered_image_to_use is not None:

            # Create a local copy of the filtered image
            local_filtered_image: ImageData = copy(filtered_image_to_use)

            # Find the outer level key of the oldest image
            newest_image_seconds = local_filtered_image.image_capture_time.secs

            # Update the status
            new_status = WAITING_FOR_ROBOT_POSE

            # Wait until a robot pose was taken at the same time as the newest image
            if newest_image_seconds in self.list_of_robot_poses.keys():

                # Update the status
                new_status = WAITING_FOR_FORCE_READING

                # Wait until a robot force was taken at the same time as the newest image
                if newest_image_seconds in self.list_of_robot_forces.keys():

                    # Update the status
                    new_status = SEARCHING_FOR_MATCHING_DATA

                    # Copy all data locally to avoid weird threading things
                    local_list_of_robot_poses = copy(self.list_of_robot_poses[newest_image_seconds])
                    local_list_of_robot_forces = copy(self.list_of_robot_forces[newest_image_seconds])

                    # Pull out the nanoseconds stamp of the most recently filtered image
                    newest_image_nanoseconds = local_filtered_image.image_capture_time.nsecs

                    # Find the closest nanoseconds keys of the pose and the force if they exist
                    closest_pose_nanoseconds = self.find_closest_key_value(newest_image_seconds,
                                                                           newest_image_nanoseconds,
                                                                           {
                                                                               newest_image_seconds: local_list_of_robot_poses})

                    closest_force_nanoseconds = self.find_closest_key_value(newest_image_seconds,
                                                                            newest_image_nanoseconds,
                                                                            {
                                                                                newest_image_seconds: local_list_of_robot_forces})

                    # If a pose and a force were found that match the image
                    if closest_pose_nanoseconds is not None and closest_force_nanoseconds is not None:

                        # Update the status
                        new_status = MATCHING_DATA_FOUND

                        if self.register_new_data_flag:

                            # Update the status
                            new_status = REGISTERING_NEW_DATA

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
                            self.filtered_image_for_registering_data = None
                            self.register_new_data_flag = False

                        elif self.save_valid_positions:

                            # Update the status
                            new_status = SAVING_VALID_POSE

                            # Save the position and mask
                            self.last_valid_positions.append(
                                local_list_of_robot_poses[closest_pose_nanoseconds][OBJECT])
                            self.last_valid_masks.append(local_filtered_image.image_mask)

                            # Maintain the length of the lists
                            for this_list in [self.last_valid_positions, self.last_valid_masks]:
                                if len(this_list) > self.max_num_valid_data_points:
                                    this_list.pop(0)

                            # Clear the saved filtered image
                            self.filtered_image_for_saving_valid_positions = None

        self.publish_node_status(new_status=new_status, delay_publishing=0.25, default_status=default_status)


if __name__ == '__main__':
    # create node object
    node = ImagePositionRegistrationNode()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    while not is_shutdown():
        node.main_loop()

    print("Node terminated.")
