"""
Contains code for bridge_list_of_points_multi_array function.
"""

# Import standard libraries
import cv2
from numpy import sum, uint8, array, zeros, delete, append, ndarray
from thyroid_ultrasound_imaging_support.ImageData.BridgeImageDataMessageConstants import *
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension


def bridge_list_of_points_multi_array(direction: int,
                                      list_of_points: list = None, array_message: UInt16MultiArray = None):
    """
    Converts a 1D list of (x, y) coordinates to a ROS Multi-array or vice versa.

    Parameters
    ----------
    direction
        Determines whether a message object or a list object will be created.

    list_of_points
        A list of points to convert into a ROS message. Each (x, y) coordinate is stored as a tuple.

    array_message
        A ROS message to convert into a list of points.
    """

    # Ensure enough arguments are passed to the function
    if list_of_points is None and array_message is None:
        raise Exception("Both list_of_points and array_message are None.")

    if direction == TO_MESSAGE:

        # Verify that the proper input is not None
        if list_of_points is None:
            raise Exception("list_of_points cannot be None when converting to message.")

        # Create an empty message
        return_message = UInt16MultiArray()

        # Return the empty message if the list is empty
        if list_of_points is None or len(list_of_points) == 0:
            return return_message

        # Check the type of the input list
        if type(list_of_points) is list:

            # Convert the list of points to a numpy array
            list_of_points = array(list_of_points)

        # Get the shape of the list of points
        if type(list_of_points) is ndarray:
            shape = list_of_points.shape

        # Raise an exception if the list is not of type array
        else:
            raise Exception("List type not recognized.")

        # Check that the array has the correct dimensions
        if not shape[1] == 2:
            raise Exception("List of points is formatted incorrectly.")

        # Reshape the array to be a 1D list
        return_message.data = list_of_points.reshape(shape[0] * shape[1])

        # Define the data padding to be zero
        return_message.layout.data_offset = 0

        # Define the layout of the array
        return_message.layout.dim.append(MultiArrayDimension)
        return_message.layout.dim[0].label = "coordinate pair"
        return_message.layout.dim[0].size = shape[0]
        return_message.layout.dim[0].stride = shape[0] * shape[1]

        return_message.layout.dim.append(MultiArrayDimension)
        return_message.layout.dim[1].label = "direction"
        return_message.layout.dim[1].size = shape[1]
        return_message.layout.dim[1].stride = 0

        # Return the message object
        return return_message

    elif direction == TO_OBJECT:

        # Verify that the proper input is not None
        if array_message is None:
            raise Exception("array_message cannot be None when converting to message.")

        # Create an empty array
        # return_array = array([[0, 0]])

        # Create an empty list
        return_list = []

        # Iterate through the message data using the index of the first coordinate in each coordinate pair
        for i in range(0, len(array_message.data), 2):

            # Calculate the correct location where each data point should be stored
            # column = int(i % array_message.layout.dim[1].size)
            # row = int((i - column) / array_message.layout.dim[1].size)

            # Append the data from the flattened array onto the 2D array
            # return_array = append(return_array, array([[array_message.data[i], array_message.data[i + 1]]]), axis=0)

            # Append the data from the flattened array onto the 2D list
            return_list.append((array_message.data[i], array_message.data[i + 1]))

        # Delete dummy data entered in first row
        # return delete(return_array, 0, 0)
        return return_list

    else:
        raise Exception("Direction is not recognized.")


if __name__ == "__main__":

    # Create a list of points to test
    trial_list = [(1, 2), (3, 4), (5, 6), (7, 8)]
    trial_list = []

    # Generate the corresponding message
    message_result = bridge_list_of_points_multi_array(TO_MESSAGE, trial_list)

    # Generate the corresponding object to the generated list
    object_result = bridge_list_of_points_multi_array(TO_OBJECT, array_message=message_result)

    # Print the result of the test
    print("Test result: " + str(trial_list == object_result))
