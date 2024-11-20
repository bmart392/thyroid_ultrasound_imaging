"""
Contains the code for the find_closest_key_value function.
"""


def find_closest_key_value(seconds_key_to_search_for: int, nanoseconds_key_to_search_for: int,
                           dict_to_search_in: dict) -> int:
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
    """

    # Create a variable to store the resulting nanosecond key of the inner dict
    closest_result_nanoseconds = None

    # If a lower level dictionary exists with the given second key,
    if dict_to_search_in.get(seconds_key_to_search_for) is not None:

        # Update the result to allow for comparison
        closest_result_nanoseconds = 0

        # For each nanosecond pose key in that lower level dictionary,
        for nanoseconds_key in dict_to_search_in.get(seconds_key_to_search_for).keys():

            # Check to see if it is the closest pose to the image
            if abs(nanoseconds_key_to_search_for - nanoseconds_key) < \
                    abs(nanoseconds_key_to_search_for - closest_result_nanoseconds):
                # If so, save the pose
                closest_result_nanoseconds = nanoseconds_key

    return closest_result_nanoseconds
