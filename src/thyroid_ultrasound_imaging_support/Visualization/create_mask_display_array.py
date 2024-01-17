"""
Contains the code for the create_mask_display_array function.
"""

# Import standard python packages
from numpy import array, uint8


def create_mask_display_array(mask: array, multiplier: int = 255):
    """
    Modifies a mask array so that it can be properly displayed.

    Parameters
    ----------
    mask
        a numpy array of the mask to display
    multiplier
        the value that all values in the mask will be multiplied by.
    """
    return uint8(mask * multiplier)
