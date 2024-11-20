"""
File containing the update_dict_with_force function definition.
"""

# Import standard python packages
from typing import Dict

# Import standard ROS packages
from geometry_msgs.msg import WrenchStamped

# Import custom python packages
from thyroid_ultrasound_imaging_support.RegisteredData.update_dict_with_key_value_pair import \
    update_dict_with_key_value_pair
from thyroid_ultrasound_imaging_support.RegisteredData.RobotForce import RobotForce


def update_dict_with_force(force_msg: WrenchStamped, message_time_secs: float, message_time_nsecs: float,
                           dict_to_update: Dict[float, Dict[float, tuple]]):
    """
    Updates the given dictionary, formatted as shown, with the given force data.

    Parameters
    ----------
    force_msg :
        The msg containing the force data to be saved.
    message_time_secs :
        The time stamp of the force message in seconds.
    message_time_nsecs :
        The time stamp of the force message in nanoseconds.
    dict_to_update :
        The dictionary in which the force data is to be stored.
    """

    # Add the object to the dictionary
    update_dict_with_key_value_pair((RobotForce(source_message=force_msg), force_msg),
                                    message_time_secs, message_time_nsecs, dict_to_update)


if __name__ == '__main__':
    pass
