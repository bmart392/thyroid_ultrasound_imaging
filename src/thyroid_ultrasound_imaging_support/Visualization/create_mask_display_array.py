"""
Contains the code for the create_mask_display_array function.
"""

# Import standard python packages
from numpy import array, uint8
from cv2 import cvtColor, COLOR_GRAY2RGB, COLOR_GRAY2BGR

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageFilter.FilterConstants import COLOR_GRAY, COLOR_BGR, COLOR_RGB


def create_mask_display_array(mask: array, multiplier: int = 255,
                              output_color_space: int = COLOR_GRAY):
    """
    Modifies a mask array so that it can be properly displayed.

    Parameters
    ----------
    mask
        a numpy array of the mask to display
    multiplier
        the value that all values in the mask will be multiplied by.
    output_color_space
        the color space that will be used to display the result
    """
    result_mask = uint8(mask * multiplier)

    # Recolor to RGB if necessary
    if output_color_space == COLOR_RGB:
        result_mask = cvtColor(result_mask, COLOR_GRAY2RGB)

    # Recolor to BGR if necessary
    elif output_color_space == COLOR_BGR:
        result_mask = cvtColor(result_mask, COLOR_GRAY2BGR)

    elif output_color_space != COLOR_GRAY:
        raise Exception("Output color space of " + str(output_color_space) + " was not recognized.")

    return result_mask
