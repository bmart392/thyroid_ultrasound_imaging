#!/usr/bin/env python3

"""
File containing the RegisteredData class definition.
"""

# Import standard python packages

# Import standard ROS packages
from geometry_msgs.msg import WrenchStamped

# Import custom python packages
from thyroid_ultrasound_imaging_support.RegisteredData.MessageCompatibleObject import *

# Import custom ROS packages

# Define constants for the names of the data fields in the object
FORCE_X: str = 'Force X'
FORCE_Y: str = 'Force Y'
FORCE_Z: str = 'Force Z'
TORQUE_X: str = 'Torque X'
TORQUE_Y: str = 'Torque Y'
TORQUE_Z: str = 'Torque Z'

# Define a constant for the name of the file generated when this object is saved
ROBOT_FORCE_FILE_NAME: str = 'RobotForce' + TEXT_FILE_EXTENSION


class RobotForce(MessageCompatibleObject):
    def __init__(self, robot_force: list = None,
                 source_message: WrenchStamped = None,
                 source_file_path: str = None):
        """
        Creates a RobotForce object and fills in any fields given.

        Parameters
        ----------
        robot_force :
            A specific robot force measurement that can be used to populate the object.
        source_message :
            A message containing a robot force measurement that can be used to populate the object.
        source_file_path :
            The absolute path to file containing RobotPose data.
        """
        # Call the constructor of the super class
        super(RobotForce, self).__init__(source_message=source_message,
                                         source_file_path=source_file_path)

        # Pull in the new robot force if a source message was not given,
        if source_message is None and source_file_path is None:
            self.robot_force_x = robot_force[0]
            self.robot_force_y = robot_force[1]
            self.robot_force_z = robot_force[2]
            self.robot_torque_x = robot_force[3]
            self.robot_torque_y = robot_force[4]
            self.robot_torque_z = robot_force[5]

    def convert_object_message(self, direction: str, message=None):
        """
        Converts between RobotForce objects and Float64Stamped messages based on the direction given.

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
            new_msg = WrenchStamped()

            # Fill out the appropriate fields
            new_msg.header.stamp = self.time_stamp
            new_msg.wrench.force.x = self.robot_force_x
            new_msg.wrench.force.y = self.robot_force_y
            new_msg.wrench.force.z = self.robot_force_z
            new_msg.wrench.torque.x = self.robot_torque_x
            new_msg.wrench.torque.y = self.robot_torque_y
            new_msg.wrench.torque.z = self.robot_torque_z

            # Return the new message
            return new_msg

        elif direction == TO_OBJECT:
            # Ensure that the message field is not empty
            if message is None:
                raise Exception("No message was given from which to build an object.")

            # Declare the type of message
            message: WrenchStamped

            # Pull out the relevant data
            self.time_stamp = message.header.stamp
            self.robot_force_x = message.wrench.force.x
            self.robot_force_y = message.wrench.force.y
            self.robot_force_z = message.wrench.force.z
            self.robot_torque_x = message.wrench.torque.x
            self.robot_torque_y = message.wrench.torque.y
            self.robot_torque_z = message.wrench.torque.z

        else:
            raise Exception(direction + " is not a recognized direction.")

    def load_class_specific_attributes(self, single_lines: list):
        """
        Rebuild the data specific to the RobotForce class contained in the list of strings given.

        Parameters
        ----------
        single_lines :
            A list of single line string representations of the data required to build a RobotForce object.
        """

        # For each single line
        for single_line in single_lines:

            # Pull out the field name and value
            field_name, value = rebuild_data(single_line)

            if field_name == FORCE_X:
                self.robot_force_x = value
            elif field_name == FORCE_Y:
                self.robot_force_y = value
            elif field_name == FORCE_Z:
                self.robot_force_z = value
            elif field_name == TORQUE_X:
                self.robot_torque_x = value
            elif field_name == TORQUE_Y:
                self.robot_torque_y = value
            elif field_name == TORQUE_Z:
                self.robot_torque_z = value

            # Raise an exception if the field name is not recognized
            else:
                raise Exception(field_name + " is not a recognized field of the RobotForce class.")

    def save_load(self, action: str, path_to_file_location: str) -> str:
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
            # Add each class parameter as a line of text
            data_string = create_single_line_simple_data(FORCE_X, self.robot_force_x, "")
            data_string = create_single_line_simple_data(FORCE_Y, self.robot_force_y, data_string)
            data_string = create_single_line_simple_data(FORCE_Z, self.robot_force_z, data_string)
            data_string = create_single_line_simple_data(TORQUE_X, self.robot_torque_x, data_string)
            data_string = create_single_line_simple_data(TORQUE_Y, self.robot_torque_y, data_string)
            data_string = create_single_line_simple_data(TORQUE_Z, self.robot_torque_z, data_string)

            # Add the time stamp and create the file
            return self.save_to_file(path_to_file_location, ROBOT_FORCE_FILE_NAME, data_string)

        elif action == LOAD_FILE:
            self.load_from_file(path_to_file_location)
        else:
            raise Exception(action + " is not a recognized action.")
