"""
File containing the code for a basic node that allows logging of messages.
"""

# Import standard ROS packages
from rospy import init_node, spin, Rate, Subscriber, is_shutdown, Publisher, get_name

# Import custom ROS packages
from thyroid_ultrasound_high_level_control.msg import log_message

# Import custom python packages
from thyroid_ultrasound_imaging_support.RosSupport.LoggingConstants import *


class BasicNode:
    def __init__(self):

        # Define a common publisher to send the log messages
        self.logger = Publisher('/system/logging', log_message, queue_size=1)

    def log_single_message(self, message: str, verbosity: int):

        # Create a new log message
        message_to_send = log_message()

        # Fill in the fields in the message to send
        message_to_send.message = message
        message_to_send.verbosity = verbosity
        message_to_send.source = get_name()

        # Publish the new message
        self.logger.publish(message_to_send)

