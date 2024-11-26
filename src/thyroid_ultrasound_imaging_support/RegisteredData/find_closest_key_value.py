"""
Contains the code for the find_closest_key_value function.
"""


def find_closest_key_value(seconds_key_to_search_for: int, nanoseconds_key_to_search_for: int,
                           dict_to_search_in: dict, additional_offset_steps: int = 0):
    """
    Returns the data point contained in a {second_key : {nano_second_key : data, ...}, ...} if the given second
    and nanosecond keys are contained in the given dictionary.

    Parameters
    ----------
    seconds_key_to_search_for
        The outer level key to search for in the dictionary.
    nanoseconds_key_to_search_for
        The inner level key to search for in the lower level dictionary.
    dict_to_search_in
        The two-tier dictionary in which to search.
    additional_offset_steps
        The number of steps to take away from the given index in each direction.

    Returns
    -------
    int or (int, int)

    """

    # Create a variable to store the resulting nanosecond key of the inner dict
    closest_result_seconds = None
    closest_result_nanoseconds = None

    # For each additional step to take in each direction,
    for index_offset in range(0 - additional_offset_steps, 1 + additional_offset_steps):

        # Calculate the adjusted second key to search for
        adjusted_seconds_key_to_search_for = seconds_key_to_search_for + index_offset

        # If a lower level dictionary exists with the given second key,
        if dict_to_search_in.get(adjusted_seconds_key_to_search_for) is not None:

            # For each nanosecond pose key in that lower level dictionary,
            for nanoseconds_key in dict_to_search_in.get(adjusted_seconds_key_to_search_for).keys():

                # Adjust the nanosecond key based on the offset index
                adjusted_nanoseconds_key = nanoseconds_key + (10**9 * index_offset)

                try:
                    # Check to see if it is the closest pose to the image
                    if abs(nanoseconds_key_to_search_for - adjusted_nanoseconds_key) < \
                            abs(nanoseconds_key_to_search_for - closest_result_nanoseconds):
                        # If so, save the pose
                        closest_result_seconds = adjusted_seconds_key_to_search_for
                        closest_result_nanoseconds = nanoseconds_key

                except TypeError:  # If the closest result hasn't been set yet, set it as the newest value
                    closest_result_seconds = adjusted_seconds_key_to_search_for
                    closest_result_nanoseconds = nanoseconds_key

    if additional_offset_steps == 0:
        return closest_result_nanoseconds
    else:
        return closest_result_seconds, closest_result_nanoseconds
