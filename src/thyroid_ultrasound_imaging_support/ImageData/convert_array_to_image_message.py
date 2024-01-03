"""
Contains definition for convert_array_to_image_message function
"""

# Import from standard packages
from numpy import array
from rospy import Time
from cv_bridge import CvBridge

# Import from ROS standard packages
from sensor_msgs.msg import Image


def convert_array_to_image_message(image: array) -> Image:
    """
    Converts an array containing an image to a sensor_msgs/Image message timestamped with the current time.

    Parameters
    ----------
    image
        The numpy array containing the image to send
    """

    # Generate an image message to publish
    resulting_image_message: Image = CvBridge().cv2_to_imgmsg(image, encoding="passthrough")

    # Register when the image was taken
    resulting_image_message.header.stamp = Time.now()

    return resulting_image_message
