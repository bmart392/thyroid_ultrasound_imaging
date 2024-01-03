"""
Contains definition for convert_image_message_to_array function
"""

# Import from standard packages
from numpy import frombuffer, reshape, uint8, array

# Import from ROS standard packages
from sensor_msgs.msg import Image


def convert_image_message_to_array(data: Image) -> array:
    """
    Converts a sensor_msgs/Image message to a numpy array.
    """

    # Read in the data
    image_array = frombuffer(data.data, dtype=uint8)

    # Reshape it according to its encoding
    if data.encoding == "8UC1":
        return reshape(image_array, (data.height, data.width))
    elif data.encoding == "8UC3":
        return reshape(image_array, (data.height, data.width, 3))
    else:
        raise Exception("Image encoding of " + str(data.encoding) + " is not compatible.")
