#!/usr/bin/env python3

"""
File containing the RegisteredData class definition.
"""

# Import standard python packages
from os import mkdir, walk

# Import standard ROS packages

# Import custom python packages
from thyroid_ultrasound_imaging_support.RegisteredData.MessageCompatibleObject import *
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData, IMAGE_DATA_OBJECT
from thyroid_ultrasound_imaging_support.RegisteredData.RobotPose import RobotPose, ROBOT_POSE_FILE_NAME
from thyroid_ultrasound_imaging_support.RegisteredData.RobotForce import RobotForce, ROBOT_FORCE_FILE_NAME
from thyroid_ultrasound_imaging_support.Validation.date_stamp_str import date_stamp_str

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import RegisteredDataMsg

# Define a constant to use to name the folder
REGISTERED_DATA_FOLDER_NAME: str = 'RegisteredData_'
NUM_CONTENTS_IN_REGISTERED_DATA_FOLDER: int = int(3)


class RegisteredData(MessageCompatibleObject):
    def __init__(self,
                 image_data_object: ImageData = None,
                 robot_pose: RobotPose = None,
                 robot_force: RobotForce = None,
                 source_message: RegisteredDataMsg = None,
                 source_file_path: str = None
                 ):
        """
        Creates a RegisteredData object and fills in any fields given.

        Parameters
        ----------
        image_data_object :
            An image data object that can be used to populate the object.
        robot_pose :
            A robot pose object that can be used to populate the object.
        robot_force :
            A robot force object that can be used to populate the object.
        source_message :
            A message containing a robot pose that can be used to populate the object.
        source_file_path :
            The absolute path to file containing RegisteredData data.
        """
        # Call the constructor of the super class
        super(RegisteredData, self).__init__(source_message=source_message,
                                             source_file_path=source_file_path)

        # Pull in the new information if a source message is not given
        if source_message is None and source_file_path is None:

            # Set the timestamp of the object as the timestamp of the ImageDataObject
            if image_data_object is not None and image_data_object.image_capture_time is not None:
                self.time_stamp = image_data_object.image_capture_time

            # Save the parameters
            self.image_data = image_data_object
            self.robot_pose = robot_pose
            self.robot_force = robot_force

    def convert_object_message(self, direction: str, message=None):
        """
        Converts between RegisteredData objects and RegisteredDataMsg messages based on the direction given.

        Parameters
        ----------
        direction :
            The direction in which to convert.
        message :
            The message to use to build a new object, if given.
        Returns
        -------
        RegisteredDataMsg
            The object converted to a message if the direction is TO_MESSAGE
        """
        if direction == TO_MESSAGE:
            # Make a new message
            new_msg = RegisteredDataMsg()

            # Fill out the appropriate data only if it is not None
            new_msg.header.stamp = self.time_stamp
            if self.image_data is not None:
                new_msg.image_data = self.image_data.bridge_image_data_and_message(direction=TO_MESSAGE)
            if self.robot_pose is not None:
                new_msg.robot_pose = self.robot_pose.convert_object_message(direction=TO_MESSAGE)
            if self.robot_force is not None:
                new_msg.robot_force = self.robot_force.convert_object_message(direction=TO_MESSAGE)

            # Return the new message
            return new_msg

        elif direction == TO_OBJECT:
            # Ensure that the message field is not empty
            if message is None:
                raise Exception("No message was given from which to build an object.")

            # Declare the type of message
            message: RegisteredDataMsg

            # Pull out the relevant data
            self.time_stamp = message.header.stamp
            self.image_data = ImageData(image_data_msg=message.image_data)
            self.robot_pose = RobotPose(source_message=message.robot_pose)
            self.robot_force = RobotForce(source_message=message.robot_force)

        else:
            raise Exception(direction + " is not a recognized direction.")

    def save_load(self, action: str, path_to_file_location: str = None):
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
        # Ensure that the file path exists before continuing
        if not exists(path_to_file_location):
            raise Exception(path_to_file_location + " is not a valid file location.")

        # Make sure the file path has a backslash
        if path_to_file_location[-1] != '/':
            path_to_file_location = path_to_file_location + '/'

        if action == SAVE_OBJECT:

            # Generate the file path to use to build the individual files
            new_save_location = path_to_file_location + REGISTERED_DATA_FOLDER_NAME + date_stamp_str(suffix='/')

            # Make a directory in the new location
            mkdir(new_save_location)

            # Make the data for component
            self.image_data.save_object_to_file(save_location=new_save_location)
            self.robot_pose.save_load(action=SAVE_OBJECT, path_to_file_location=new_save_location)
            self.robot_force.save_load(action=SAVE_OBJECT, path_to_file_location=new_save_location)

            # Return the location of the newly generated data
            return new_save_location

        elif action == LOAD_FILE:

            # Generate a list of the directories included in the path location
            list_of_directories = next(walk(path_to_file_location))[1]

            # Ensure that only one folder is in the location, and it is an image data object folder
            if len(list_of_directories) > 1 or IMAGE_DATA_OBJECT not in list_of_directories[0]:
                raise Exception(path_to_file_location + " is not properly formatted for registered data.")

            # Create the image data object from the included directory
            self.image_data = ImageData(image_data_location=path_to_file_location + list_of_directories[0])

            # Copy the timestamp from the image data object
            if self.image_data is not None:
                self.time_stamp = self.image_data.image_capture_time

            # Ensure that the RobotPose file exists
            if exists(path_to_file_location + ROBOT_POSE_FILE_NAME):

                # Generate a new RobotPose object from the robot pose file
                self.robot_pose = RobotPose(source_file_path=path_to_file_location + ROBOT_POSE_FILE_NAME)

            else:
                raise Exception("The RegisteredData folder at " + path_to_file_location +
                                " does not contain a RobotPose file named " + ROBOT_POSE_FILE_NAME)

            # Ensure that the RobotForce file exists
            if exists(path_to_file_location + ROBOT_FORCE_FILE_NAME):

                # Generate a new RobotForce object form the robot force file
                self.robot_force = RobotForce(source_file_path=path_to_file_location + ROBOT_FORCE_FILE_NAME)

            else:
                raise Exception("The RegisteredData folder at " + path_to_file_location +
                                " does not contain a RobotForce file named " + ROBOT_FORCE_FILE_NAME)
        else:
            raise Exception(action + " is not a recognized action.")




