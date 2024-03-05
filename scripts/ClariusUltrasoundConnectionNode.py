#!/usr/bin/env python3

"""
File containing ImageFiterNode class definition and ROS running code.
"""

# TODO - Dream - Add proper logging stuff using the BasicNode class
# TODO - Dream - Add proper try-cath error checking everywhere and incorporate logging into it
# TODO - Dream - Add proper status publishing
# TODO - High - Use the imaging depth stored within each image data object

# Import standard python packages
from numpy import zeros, uint8, array
import ctypes
from os.path import exists, expanduser
from os import makedirs
import sys
from PIL import Image
from PySide2 import QtGui
from cv2 import imshow, cvtColor, waitKey, destroyAllWindows, COLOR_BGR2GRAY, COLOR_GRAY2BGR
from datetime import date, datetime
from matplotlib.image import imsave

# Import standard ROS packages
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *

# Define the image width and height
IMAGE_WIDTH: int = 640
IMAGE_HEIGHT: int = 480
PORT: int = 5828

# Define node behavior
IP: str = "192.168.0.101"
VISUALIZATION_INCLUDED: bool = False

# Define file saving behavior
FILE_EXTENSION: str = '.png'

# Add the folder containing the library files
sys.path.append('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/cast_libraries/')

# Import Clarius specific packages
libcast_handle = ctypes.CDLL("/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/cast_libraries/libcast.so",
                             ctypes.RTLD_GLOBAL)._handle  # load the libcast.so shared library
pyclariuscast = ctypes.cdll.LoadLibrary(
    "/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/cast_libraries/pyclariuscast.so"
)  # load the pyclariuscast.so shared library
import pyclariuscast

# Define a variable to store the received ultrasound image
processed_image = zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 4), dtype=uint8)

image_is_frozen = False


class ClariusUltrasoundConnectionNode(BasicNode):

    def __init__(self):

        # Call the constructor of the base class
        super().__init__()

        self.folder_destination = None
        self.folder_path = None
        self.saved_image_index = 1

        self.save_incoming_images = False

        # Make a new folder to store the images in if it does not already exist
        # if not exists(self.folder_path):
        #     makedirs(self.folder_path)

        # Define a variable to note if the node quit on an error
        self.error_message = None

        # Get the home path
        path = expanduser("~/")

        # Create a casting object
        self.cast = pyclariuscast.Caster(self.new_processed_image, self.new_raw_image, self.new_spectrum_image,
                                         self.freeze_function, self.buttons_function)

        # Initialize the casting
        ret = self.cast.init(path, IMAGE_WIDTH, IMAGE_HEIGHT)

        # If the initialization was successful, connect and notify the user
        if ret:
            ret = self.cast.connect(IP, PORT, "research")
            if ret:
                print("Casting connected to {0} on port {1}".format(IP, PORT))
            else:
                self.cast.destroy()
                self.error_message = "Casting connection to {0} on port {1} failed.".format(IP, PORT)
                return
        else:
            self.error_message = "Casting initialization failed."
            return

        # Initialize the ROS node for publishing the data
        init_node(CLARIUS_US_PUBLISHER, anonymous=True)

        # Define the image publisher
        self.image_publisher = Publisher(IMAGE_SOURCE, Image, queue_size=100)

        Subscriber(SAVE_IMAGES, Bool, self.save_images_callback)

        Subscriber(SAVED_IMAGES_DESTINATION, String, self.saved_images_destination_callback)

        # Define the frequency at which to publish images
        freq = 30  # CHANGE THIS LINE to change the rate at which images are published
        self.rate = Rate(freq)

    @staticmethod
    def new_processed_image(image, width, height, sz, microns_per_pixel, timestamp, angle, imu):
        global processed_image

        # Process the image received
        img = QtGui.QImage(image, width, height, QtGui.QImage.Format_ARGB32)

        # Convert the image to a numpy array
        ptr = img.constBits()
        processed_image = array(ptr).reshape((height, width, 4))
        return

    @staticmethod
    def new_raw_image(image, lines, samples, bps, axial, lateral, timestamp, jpg, rf, angle):
        return

    @staticmethod
    def new_spectrum_image(image, lines, samples, bps, period, microns_per_sample, velocity_per_sample, pw):
        return

    @staticmethod
    def freeze_function(frozen):
        global image_is_frozen
        if frozen:
            print("Image is now frozen.")
            image_is_frozen = True
        else:
            print("Image is now running.")
            image_is_frozen = False
        return

    @staticmethod
    def buttons_function(button, clicks):
        print("button pressed: {0}, clicks: {1}".format(button, clicks))
        return

    def save_images_callback(self, msg: Bool):

        self.save_incoming_images = msg.data

        if msg.data and self.folder_destination is not None:
            self.folder_path = self.folder_destination + '/' + str(date.today()) + '_' + datetime.now().strftime('%H-%M')
            if not exists(self.folder_path):
                makedirs(self.folder_path)
            self.saved_image_index = 1

    def saved_images_destination_callback(self, msg: String):

        self.folder_destination = msg.data

    def save_image(self, image: array):

        if image.sum() / (IMAGE_HEIGHT * IMAGE_WIDTH) > 25 and not image_is_frozen and self.folder_path is not None:

            # Save the image to the proper location
            imsave(self.folder_path + '/Slice_' + str(self.saved_image_index).zfill(5) + FILE_EXTENSION,
                   cvtColor(image, COLOR_GRAY2BGR))

            # Increment the image counter
            self.saved_image_index = self.saved_image_index + 1

    def main_loop(self):
        while not is_shutdown():

            # Convert the image to a properly dimensioned gray scale image
            bscan = cvtColor(processed_image[:, :, 0:3], COLOR_BGR2GRAY)

            # Create an image message from the ultrasound image
            img_msg = CvBridge().cv2_to_imgmsg(bscan, encoding='passthrough')

            # Add the time that the message was received
            img_msg.header.stamp = Time.now()

            # Publish the image
            self.image_publisher.publish(img_msg)

            # Display the image if requested
            if VISUALIZATION_INCLUDED:
                imshow('raw image', bscan)
                waitKey(1)

            if self.save_incoming_images:
                self.save_image(bscan)

            # Wait to publish the next message
            self.rate.sleep()

        # Close the cast
        self.cast.destroy()

        # Close any windows that were open
        destroyAllWindows()


if __name__ == '__main__':

    # Create the node
    node = ClariusUltrasoundConnectionNode()

    if node.error_message is not None:
        print(node.error_message)
    else:

        print("Node initialized.")
        print("Press ctrl+c to terminate.")

        node.main_loop()

        print("\nShutdown signal received.")
