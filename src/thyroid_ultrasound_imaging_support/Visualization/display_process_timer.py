"""
Contains display_process_timer function.
"""

# Import standard packages
from time import time


def display_process_timer(start_of_process_time: float, message: str, mode: bool = True,
                          return_time: bool = False):
    """
    Display the amount of time a process ran in milliseconds. Returns the current time the function was called.

    Parameters
    ----------
    start_of_process_time
        Time the process being measured started as marked with a call to time().

    message
        Description to display with the measured time.

    mode
        A boolean value of True causes the elapsed time to be displayed on the command line.

    return_time
        A boolean value of True causes the function to return the elapsed time in milliseconds.
    """
    # Calculate the elapsed time
    elapsed_time = round((time() - start_of_process_time) * 1000, 4)

    # Display the elapsed time if requested
    if mode:
        print(message + " (ms): ", elapsed_time)

    # Return the elapsed time if requested
    if return_time:
        return time(), elapsed_time

    # Otherwise just return the current time
    else:
        return time()
