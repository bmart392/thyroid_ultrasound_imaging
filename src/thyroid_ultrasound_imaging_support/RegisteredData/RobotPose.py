#!/usr/bin/env python3

"""
File containing the RegisteredData class definition.
"""

# Import standard python packages
from numpy import ndarray

# Import standard ROS packages
from std_msgs.msg import MultiArrayDimension
from thyroid_ultrasound_imaging_support.ImageData.single_line_representations import create_single_line_array_data

# Import custom python packages
from thyroid_ultrasound_imaging_support.RegisteredData.MessageCompatibleObject import *

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import Float64MultiArrayStamped

# Define constants for the names of the data fields in the object
ROBOT_POSE: str = 'RobotPose'

# Define a constant for the name of the file generated when this object is saved
ROBOT_POSE_FILE_NAME: str = ROBOT_POSE + TEXT_FILE_EXTENSION


class RobotPose(MessageCompatibleObject):
    def __init__(self, robot_pose: ndarray = None,
                 source_message: Float64MultiArrayStamped = None,
                 source_file_path: str = None):
        """
        Creates a RobotPose object and fills in any fields given.

        Parameters
        ----------
        robot_pose :
            A specific robot pose that can be used to populate the object.
        source_message :
            A message containing a robot pose that can be used to populate the object.
        source_file_path :
            The absolute path to file containing RobotPose data.
        """
        # Call the constructor of the super class
        super(RobotPose, self).__init__(source_message=source_message,
                                        source_file_path=source_file_path)

        # Pull in the new robot pose if a source message is not given,
        if source_message is None and source_file_path is None:
            self.robot_pose = robot_pose

    def convert_object_message(self, direction: str, message=None):
        """
        Converts between RobotPose objects and Float64MultiArrayStamped messages based on the direction given.

        Parameters
        ----------
        direction :
            The direction in which to convert.
        message :
            The message to use to build a new object, if given.
        Returns
        -------
        Float64Stamped
            The object converted to a message if the direction is TO_MESSAGE
        """
        if direction == TO_MESSAGE:
            # Make a new message
            new_msg = Float64MultiArrayStamped()

            # Fill out the appropriate data
            new_msg.header.stamp = self.time_stamp
            new_msg.data.layout.data_offset = 0
            new_msg.data.layout.dim.append(MultiArrayDimension())
            new_msg.data.layout.dim[0].label = "row"
            new_msg.data.layout.dim[0].size = 4
            new_msg.data.layout.dim[0].stride = 16
            new_msg.data.layout.dim.append(MultiArrayDimension())
            new_msg.data.layout.dim[1].label = "column"
            new_msg.data.layout.dim[1].size = 4
            new_msg.data.layout.dim[1].stride = 0
            new_msg.data.data = self.robot_pose.reshape(16)

            # Return the new message
            return new_msg

        elif direction == TO_OBJECT:
            # Ensure that the message field is not empty
            if message is None:
                raise Exception("No message was given from which to build an object.")

            # Declare the type of message
            message: Float64MultiArrayStamped

            # Pull out the relevant data
            self.time_stamp = message.header.stamp
            self.robot_pose = array(message.data.data).reshape((4, 4))

        else:
            raise Exception(direction + " is not a recognized direction.")

    def load_class_specific_attributes(self, single_lines: list):
        """
        Rebuild the data specific to the RobotPose class contained in the list of strings given.

        Parameters
        ----------
        single_lines :
            A list of single line string representations of the data required to build a RobotForce object.
        """

        # For each single line
        for single_line in single_lines:

            # Pull out the field name and value
            field_name, value = rebuild_data(single_line)

            if field_name == ROBOT_POSE:
                self.robot_pose = value

            # Raise an exception if the field name is not recognized
            else:
                raise Exception(field_name + " is not a recognized field of the RobotPose class.")

    def save_load(self, action: str, path_to_file_location: str):
        """
        Saves the object to a text file in the given location or
        loads the object from a text file at the given location.
        Parameters
        ----------
        action :
            The flag denoting whether data will be saved or loaded.
        path_to_file_location :
            The location where the file should be saved or the path to the file from which to load the data.

        Returns
        -------
            The path to the text file generated, if saving data.
        """
        if action == SAVE_OBJECT:
            return self.save_to_file(path_to_file_location, ROBOT_POSE_FILE_NAME,
                                     create_single_line_array_data(ROBOT_POSE, self.robot_pose, ""))
        elif action == LOAD_FILE:
            self.load_from_file(path_to_file_location)
        else:
            raise Exception(action + " is not a recognized action.")
