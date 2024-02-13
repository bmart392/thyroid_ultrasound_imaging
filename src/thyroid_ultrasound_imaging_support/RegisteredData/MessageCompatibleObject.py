#!/usr/bin/env python3

"""
File containing the MessageCompatibleObject class definition.
"""

# Import standard python packages
from rospy import Time
from os.path import exists

# Import standard ROS packages

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageData.single_line_representations import create_single_line_time_stamp, \
    NEW_LINE, rebuild_data, create_single_line_simple_data

# Import custom ROS packages

# Define constants for the names of the fields of the object
TIME_STAMP: str = 'Timestamp'

# Define constants used to convert the object
TO_MESSAGE: str = 'To message'
TO_OBJECT: str = 'To object'
SAVE_OBJECT: str = 'Save object'
LOAD_FILE: str = 'Load from file'

# Define the file extension to use to save the data
TEXT_FILE_EXTENSION: str = '.txt'


class MessageCompatibleObject:
    def __init__(self, source_message=None, source_file_path: str = None):
        """
        Creates a MessageCompatibleObject and sets the timestamp for the object as the current time.

        Parameters
        ----------
        source_message :
            A message object to use to populate the object.
        source_file_path :
            A path to a file containing a MessageCompatibleObject data.
        """
        # Build the object from the message if a message is given
        if source_message is not None and source_file_path is None:
            self.convert_object_message(direction=TO_OBJECT, message=source_message)
        # Build the object from a location if a location is given
        elif source_message is None and source_file_path is not None:
            self.save_load(action=LOAD_FILE, path_to_file_location=source_file_path)
        # Otherwise just populate the time
        else:
            self.time_stamp = Time.now()

    def convert_object_message(self, direction: str, message=None):
        raise Exception("This function was not implemented in the child class.")

    def save_load(self, action: str, path_to_file_location: str) -> str:
        raise Exception("This function was not implemented in the child class.")

    def load_class_specific_attributes(self, single_lines: list):
        raise Exception("This function was not implemented in the child class.")

    def save_to_file(self, file_path: str, file_name: str, data_to_write: str) -> str:
        """
        Writes the given string to a text file in the given path with the given name.

        Parameters
        ----------

        file_path :
            The absolute path to the location where the text file should be saved
        file_name :
            A string to name the file when creating it.
        data_to_write :
            A string representation of the data to include in the text file excluding the timestamp.

        Returns
        -------
        str
            The full path to the file generated.
        """

        # Check that the file path exists before any progress is made
        if not exists(file_path):
            raise Exception(file_path + " is not a valid location.")

        # Ensure that the file path ends with a back-slash
        if file_path[-1] != '/':
            file_path = file_path + '/'

        # Add the timestamp to the data to write in the file
        data_to_write = create_single_line_time_stamp(TIME_STAMP, self.time_stamp, "") + data_to_write

        # Write the string to the text file and then save it
        absolute_file_path = file_path + file_name
        data_file = open(absolute_file_path, 'w')
        data_file.write(data_to_write)
        data_file.close()

        # Return the specific path to the generated file
        return absolute_file_path

    def load_from_file(self, path_to_file: str):
        """
        Loads the data in a given text file.

        Parameters
        ----------
        path_to_file :
            The absolute path to the specific file containing the data to load.
        """

        # Check that the given location exists before any progress is made
        if not exists(path_to_file):
            raise Exception(path_to_file + " is not a valid location.")

        # Ensure that the path leads to a text file
        if path_to_file[-len(TEXT_FILE_EXTENSION):] != TEXT_FILE_EXTENSION:
            raise Exception(path_to_file[-len(TEXT_FILE_EXTENSION)] + " is not the expected file type of " + TEXT_FILE_EXTENSION)

        # Read in every line in the file
        all_data_lines = open(path_to_file, 'r').read()

        # Remove the carriage return character at the end of the string
        all_data_lines = all_data_lines[:-len(NEW_LINE)]

        # Pull each field from the string by splitting on the carriage returns
        simple_data_lines = all_data_lines.split(NEW_LINE)

        # Fill out the time stamp from the text file
        _, self.time_stamp = rebuild_data(simple_data_lines[0])

        # Load the remainder of the data in the class specific function
        self.load_class_specific_attributes(simple_data_lines[1:])




