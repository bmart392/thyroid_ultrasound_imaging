"""
Contains the code for the update_dict_with_key_value_pair function.
"""

# Import standard python packages
from typing import Dict

# Import custom python packages


def update_dict_with_key_value_pair(data_to_store: tuple, message_time_secs: float,
                                    message_time_nsecs: float, dict_to_update: Dict[float, Dict[float, tuple]]):
    """
    Updates the given dictionary, formatted as shown, with the given value.

    Parameters
    ----------
    data_to_store :
        A tuple containing the data to store as both an object (in position 0) and as a ROS message (in position 1).
    message_time_secs :
        The time stamp of the data in seconds.
    message_time_nsecs :
        The time stamp of the data in nanoseconds.
    dict_to_update :
        The dictionary in which the data is to be stored.
    """

    try:
        # Try to add the new data into the existing place in the dictionary
        dict_to_update[message_time_secs].update({message_time_nsecs: data_to_store})

    except KeyError:

        # If the correct place in the dictionary does not exist yet, create it and then add the value
        dict_to_update.update({message_time_secs: {message_time_nsecs: data_to_store}})


if __name__ == '__main__':
    pass
