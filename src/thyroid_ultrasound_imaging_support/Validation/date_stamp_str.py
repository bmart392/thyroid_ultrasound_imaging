from datetime import datetime
from rospy import Time


def date_stamp_str(prefix: str = "", suffix: str = "_", src_timestamp: Time = None):
    """
    Generates a date stamp formatted as a string. This date stamp is friendly to file names.

    Parameters
    ----------
    prefix :
        Places any given characters at the front of the date stamp.
    suffix :
        Places any given characters at the front of the date stamp.
    src_timestamp :
        A time stamp to use to build the date stamp if the current time should not be used.

    Returns
    -------
    A date stamp string formatted as YYYY-MM-DD_HH-MM-SS-ffffff (f is microseconds) and the prefix and suffix
    prepended and appended respectively.
    """
    if src_timestamp is not None:
        current_time_datestamp = datetime.fromtimestamp(src_timestamp.to_time())
    else:
        current_time_datestamp = datetime.now()
    return prefix + str(current_time_datestamp.date()) + '_' + current_time_datestamp.strftime('%H-%M-%S-%f') + suffix



