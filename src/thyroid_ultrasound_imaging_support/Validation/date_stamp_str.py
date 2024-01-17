from datetime import date, datetime


def date_stamp_str(prefix: str = "", suffix: str = "_"):
    """
    Generates a date stamp formatted as a string. This date stamp is friendly to file names.

    Parameters
    ----------
    prefix :
        Places any given characters at the front of the date stamp.
    suffix :
        Places any given characters at the front of the date stamp.

    Returns
    -------
    A date stamp string formatted as YYYY-MM-DD_HH-MM-SS-ffffff (f is microseconds) and the prefix and suffix
    prepended and appended respectively.
    """
    return prefix + str(date.today()) + '_' + datetime.now().strftime('%H-%M-%S-%f') + suffix
