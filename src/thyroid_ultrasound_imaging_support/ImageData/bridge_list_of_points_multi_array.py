"""
Contains code for bridge_list_of_points_multi_array function.
"""

# Import standard libraries
from numpy import array, ndarray
from thyroid_ultrasound_imaging_support.ImageData.BridgeImageDataMessageConstants import *
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension, Float64MultiArray

# Define constants for the type of message to create
INT_ARRAY: str = 'INT'
FLOAT_ARRAY: str = 'FLOAT'

# Define the dimension of points included in the array
TWO_D: int = int(2)
THREE_D: int = int(3)
FOUR_D: int = int (4)


def bridge_list_of_points_multi_array(direction: str, list_of_points=None, array_message=None,
                                      msg_type: str = INT_ARRAY, point_dim: int = TWO_D,
                                      return_as_array: bool = False):
    """
    Converts a 1D list of (x, y) coordinates to a ROS Multi-array or vice versa.

    Parameters
    ----------
    direction
        Determines whether a message object or a list object will be created.

    list_of_points
        A list of points to convert into a ROS message. Each coordinate is stored as a tuple in a list or array.

    array_message
        A ROS message to convert into a list of points. The message can be an int or float array.

    msg_type
        The type of MultiArray message to create when converting to a message
    point_dim
        The dimension of each point included in the array.
    return_as_array
        Tells the function to return the generated list of points as a numpy array.
    """

    # Ensure enough arguments are passed to the function
    if list_of_points is None and array_message is None:
        raise Exception("Both list_of_points and array_message are None.")

    if direction == TO_MESSAGE:

        # Verify that the proper input is not None
        if list_of_points is None:
            raise Exception("list_of_points cannot be None when converting to message.")

        # Create an empty message
        if msg_type == INT_ARRAY:
            return_message = UInt16MultiArray()

        elif msg_type == FLOAT_ARRAY:
            return_message = Float64MultiArray()

        else:
            raise Exception("The message type given (" + msg_type + ") was not recognized.")

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

        exception_msg = None

        # Check that the array has the correct dimensions
        if not shape[1] == 2 and point_dim == TWO_D:
            exception_msg = str(TWO_D)

        if not shape[1] == 3 and point_dim == THREE_D:
            exception_msg = str(THREE_D)

        if not shape[1] == 4 and point_dim == FOUR_D:
            exception_msg = str(FOUR_D)

        if exception_msg is not None:
            raise Exception("List of points is formatted incorrectly. Dimension of each point is not " +
                            exception_msg + "D.")


        # Define the data padding to be zero
        return_message.layout.data_offset = 0

        # Define the layout of the array
        return_message.layout.dim.append(MultiArrayDimension())
        return_message.layout.dim[0].label = "coordinate pair"
        return_message.layout.dim[0].size = shape[0]
        return_message.layout.dim[0].stride = shape[0] * shape[1]

        return_message.layout.dim.append(MultiArrayDimension())
        return_message.layout.dim[1].label = "direction"
        return_message.layout.dim[1].size = shape[1]
        return_message.layout.dim[1].stride = 0

        # Reshape the array to be a 1D list
        return_message.data = list_of_points.reshape(shape[0] * shape[1])
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

        # If required, convert the result list to an array
        if return_as_array:
            return_list = array(return_list)

        # Delete dummy data entered in first row
        # return delete(return_array, 0, 0)
        return return_list

    else:
        raise Exception("Direction is not recognized.")


if __name__ == "__main__":

    # Create a list of points to test
    trial_list = [(int(1), int(2)), (int(3), int(4)), (int(5), int(6)), (int(7), int(8))]
    # trial_list = [(1, 2), (3, 4), (5, 6), (7, 8)]
    # trial_list = []

    # Generate the corresponding message
    message_result = bridge_list_of_points_multi_array(TO_MESSAGE, trial_list)

    # Generate the corresponding object to the generated list
    object_result = bridge_list_of_points_multi_array(TO_OBJECT, array_message=message_result)

    # Print the result of the test
    print("Test result: " + str(trial_list == object_result))
