"""
File containing the update_dict_with_pose function definition.
"""

# Import standard python packages
from numpy import ndarray
from typing import Dict

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageData.bridge_list_of_points_multi_array import \
    bridge_list_of_points_multi_array, FLOAT_ARRAY, FOUR_D
from thyroid_ultrasound_imaging_support.ImageData.BridgeImageDataMessageConstants import TO_MESSAGE
from thyroid_ultrasound_imaging_support.RegisteredData.update_dict_with_key_value_pair import \
    update_dict_with_key_value_pair
from thyroid_ultrasound_imaging_support.RegisteredData.RobotPose import RobotPose


def update_dict_with_pose(pose_as_matrix: ndarray, message_time_secs: float, message_time_nsecs: float,
                          dict_to_update: Dict[float, Dict[float, tuple]]):
    """
    Updates the given dictionary, formatted as shown, with the given pose data.

    Parameters
    ----------
    pose_as_matrix :
        The matrix containing the robot pose data to be saved.
    message_time_secs :
        The time stamp of the robot pose in seconds.
    message_time_nsecs :
        The time stamp of the robot pose in nanoseconds.
    dict_to_update :
        The dictionary in which the robot pose data is to be stored.
    """

    # Convert the pose array to a message
    pose_as_multi_array = bridge_list_of_points_multi_array(direction=TO_MESSAGE,
                                                            list_of_points=pose_as_matrix,
                                                            msg_type=FLOAT_ARRAY,
                                                            point_dim=FOUR_D)

    # Add the object to the dictionary
    update_dict_with_key_value_pair((RobotPose(robot_pose=pose_as_matrix), pose_as_multi_array),
                                    message_time_secs, message_time_nsecs, dict_to_update)


if __name__ == '__main__':
    pass
