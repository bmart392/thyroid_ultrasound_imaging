from time import time


def display_process_timer(start_of_process_time, message) -> float:
    """
    Display the amount of time a process ran in milliseconds.

    Parameters
    ----------
    start_of_process_time : float
        time the process being measured started as marked with a call to time().

    message: str
        Description to display with the measured time.
    """
    print(message + " (ms): ", round((time() - start_of_process_time) * 1000, 4))
    return time()
