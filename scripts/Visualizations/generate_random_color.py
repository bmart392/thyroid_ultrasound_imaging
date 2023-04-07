"""
Contains generate_random_color function.
"""
# Import standard packages
from random import random
from numpy import uint8


def generate_random_color(num_channels: int = 3, color_value_type: type = uint8):
    """
    Generate a random color.

    Parameters
    ----------
    num_channels
        the number of channels for the color. Currently only 1 and 3 channel colors are supported.
    color_value_type
        the type of value that the color will be stored as. Currently only float and uint8 are supported.
    """

    # Select the maximum color value based on the color value type
    # Raise an exception if the color value type is not recognized
    if color_value_type == uint8:
        max_value = 255
    elif color_value_type == float:
        max_value = 1
    else:
        raise Exception("Image value type not recognized. Only float or uint8 accepted.")

    # Create a color based on the number of channels in the image and the maximum color value selected above
    # Raise an exception if the number of channels in the image is not recognized
    if num_channels == 3:
        color = [1, 1, 1]
        for i in range(len(color)):
            color[i] = round(color[i] * random() * max_value)
    elif num_channels == 1:
        color = random() * max_value
    else:
        raise Exception('Number of channels not recognized. Only 1 or 3 channel images accepted.')

    # Return a tuple as the color
    return tuple(color)
