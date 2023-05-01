"""
Contains display_process_timer function.
"""

# Import standard packages
from time import time


def display_process_timer(start_of_process_time: float, message: str, mode: bool = True) -> float:
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
    """
    if mode:
        print(message + " (ms): ", round((time() - start_of_process_time) * 1000, 4))
    return time()
