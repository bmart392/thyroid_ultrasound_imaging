"""
Contains code for bridge_list_of_contours_multi_array function.
"""

# Import standard libraries
from numpy import array
from thyroid_ultrasound_imaging_support.ImageData.BridgeImageDataMessageConstants import *
from thyroid_ultrasound_imaging_support.ImageData.bridge_list_of_points_multi_array import bridge_list_of_points_multi_array
from std_msgs.msg import UInt16MultiArray


def bridge_list_of_contours_multi_array(direction: str, list_of_contours: list = None, array_message: UInt16MultiArray = None):
    """
    Converts a 2D list of (x, y) coordinates into a ROS Multi-array message and vice versa.
    Uses a tuple of value (65535, 65535) as a separator between different contours.

    Parameters
    ----------
    direction
        Determines whether a message object or a list object will be created.

    list_of_contours
        A list of contours to convert into a ROS message. Each contour is defined as a list of (x, y) coordinates
        stored as tuples.

    array_message
        A ROS message to convert into a list of contours.
    """

    if direction == TO_MESSAGE:

        # Verify that the proper input is not None
        if list_of_contours is None:
            raise Exception("list_of_contours cannot be None when converting to message.")

        # Define one list that will contain all contours
        list_of_points_of_all_contours = []

        # Iterate through each list of contours
        for contour_list in list_of_contours:

            # Iterate through each coordinate in the list for this contour
            for coordinate_pair in contour_list:

                # Add each point to the list of all points
                list_of_points_of_all_contours.append(coordinate_pair)

            # Add a placeholder coordinate to separate each contour
            list_of_points_of_all_contours.append(PLACEHOLDER_COORDINATE)

        # Create MultiArray object using list of points function
        return bridge_list_of_points_multi_array(TO_MESSAGE, list_of_points_of_all_contours)

    elif direction == TO_OBJECT:

        # Verify that the proper input is not None
        if array_message is None:
            raise Exception("array_message cannot be None when converting to message.")

        # Convert the array_message to a single list of points that contains all contours
        list_of_points_of_all_contours = bridge_list_of_points_multi_array(TO_OBJECT, array_message=array_message,
                                                                           return_as_array=True)

        # Create empty list in which to store all the contours
        result_list_of_contours = []

        # Set a flag to know when to start filling a new list
        fill_new_list = False

        # Define an empty list to store the points of a single contour
        temp_list_of_points_for_single_contour = []

        # For each coordinate pair in the list of points of all contours,
        for coordinate_pair in list_of_points_of_all_contours:

            # If it is a placeholder value, it is time to start filling a new list
            if coordinate_pair[0] == PLACEHOLDER_COORDINATE[0] and coordinate_pair[1] == PLACEHOLDER_COORDINATE[1]:

                # Append the last list of points to the result list
                result_list_of_contours.append(array(temp_list_of_points_for_single_contour))

                # Set the flag
                fill_new_list = True

            # If it is a valid data point,
            else:

                # If the list of points for a new contour should be started,
                if fill_new_list:

                    # Overwrite the previous list
                    temp_list_of_points_for_single_contour = [coordinate_pair]

                    # Reset the flag
                    fill_new_list = False

                else:
                    # Otherwise append the current coordinate to the temporary list of points
                    temp_list_of_points_for_single_contour.append(coordinate_pair)

        # Return the list containing multiple contours
        return result_list_of_contours

    else:
        raise Exception("Direction is not recognized.")


if __name__ == "__main__":

    # Create a list of points to test
    trial_list = [[(1, 2), (3, 4), (5, 6), (7, 8)], [(9, 10)], [(11, 12), (13, 14)]]

    # Generate the corresponding message
    message_result = bridge_list_of_contours_multi_array(TO_MESSAGE, trial_list)

    # Generate the corresponding object to the generated list
    object_result = bridge_list_of_contours_multi_array(TO_OBJECT, array_message=message_result)

    print(object_result)

    # Print the result of the test
    print("Test result: " + str(trial_list == object_result))