#!/usr/bin/env python3

"""
File containing ImageFiterNode class definition and ROS running code.
"""

# Import standard packages
from numpy import frombuffer, reshape, uint8  # , ones, sign
from cv_bridge import CvBridgeError  # , CvBridge
from time import time
from copy import copy

# Import ROS packages
# import rospy
from rospy import init_node, spin, Subscriber, Publisher, signal_shutdown
from geometry_msgs.msg import TwistStamped  # , Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String  # ,Float64, String, UInt32MultiArray

# Import custom objects
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData

from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilterThreshold import ImageFilterThreshold
from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilterGrabCut import ImageFilterGrabCut
from thyroid_ultrasound_imaging_support.ImageFilter.FilterConstants import COLOR_GRAY, COLOR_BGR

from thyroid_ultrasound_imaging_support.Visualization.VisualizationConstants import *
from thyroid_ultrasound_imaging_support.Visualization.Visualization import Visualization

from thyroid_ultrasound_imaging_support.Controller.ImagePositioningController import ImagePositioningController
from thyroid_ultrasound_imaging_support.Visualization.display_process_timer import display_process_timer
from thyroid_ultrasound_imaging_support.UserInput.user_input_crop_coordinates import user_input_crop_coordinates

def request_handler(request):

    # Create an image data object from the image_data message sent
    temp_image_data = ImageData(request.sent_data)

    # Capture the image crop coordinates from the image
    image_crop_coordinates = user_input_crop_coordinates(temp_image_data)



if __name__ == '__main__':

    # Initialize the server
    init_node("Select Crop Coordinates Server")
    print("Server initialized.")

    image_filter = ImageFilterGrabCut(image_crop_included=True,
                                      image_crop_coordinates=[[0, 0], [100, 100]])

    image_data = ImageData(image_color=COLOR_BGR,
        image_filepath='/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/Test/'
                       'Images/Series2/Slice_20.png')

    image_filter.crop_image(image_data)
    image_filter.colorize_image(image_data)

    image_filter.generate_previous_mask_from_user_input(image_data)

    print("Done")
    signal_shutdown("End of work")
