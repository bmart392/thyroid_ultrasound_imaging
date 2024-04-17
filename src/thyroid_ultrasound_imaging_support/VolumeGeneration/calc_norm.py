# Import standard packages
from numpy import ndarray, array
from numpy.linalg import norm


def calc_norm(pt_1: tuple, pt_2: ndarray) -> ndarray:
    """
    Calculates the normal distance between two points.
    """
    return norm(array(pt_1) - pt_2)
