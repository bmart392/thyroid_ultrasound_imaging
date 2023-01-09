from time import time


def display_process_timer(start_of_process_time, message, print_time=True) -> float:
    """
    Display the amount of time a process ran in milliseconds.

    Parameters
    ----------
    start_of_process_time : float
        Time the process being measured started as marked with a call to time().

    message: str
        Description to display with the measured time.

    print_time: Bool
        Selector for printing the time to allow for easier debugging.
    """
    if print_time:
        print(message + " (ms): ", round((time() - start_of_process_time) * 1000, 4))
    return time()
