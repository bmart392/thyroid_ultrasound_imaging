#!/usr/bin/env python3

"""
File containing ImageFiterNode class definition and ROS running code.
"""


# Import standard python packages
from numpy import zeros, uint8, array
import ctypes
from os.path import exists, expanduser, isdir
from os import makedirs
from PySide2 import QtGui
from cv2 import imshow, cvtColor, waitKey, destroyAllWindows, COLOR_BGR2GRAY, COLOR_GRAY2BGR
from datetime import date, datetime
from matplotlib.image import imsave

# Import standard ROS packages

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_messages.msg import ImageWithTimeData

# Import custom python packages
from thyroid_ultrasound_support.MessageConversion.convert_image_array_to_image_with_time_data_message import \
    convert_image_array_to_image_with_time_data_message

# Define the image width and height
IMAGE_WIDTH: int = 640
IMAGE_HEIGHT: int = 480
PORT: int = 5828

# Define node behavior
IP: str = "192.168.0.104"
VISUALIZATION_INCLUDED: bool = False

# Define indexes for image frozen list
NEW_VALUE: int = 0
OLD_VALUE: int = 1

# Define file saving behavior
FILE_EXTENSION: str = '.png'

# Add the folder containing the library files
sys.path.append('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/cast_libraries/')

# Import Clarius specific packages
# noinspection PyProtectedMember
libcast_handle = ctypes.CDLL("/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/cast_libraries/libcast.so",
                             ctypes.RTLD_GLOBAL)._handle  # load the libcast.so shared library
pyclariuscast = ctypes.cdll.LoadLibrary("/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/"
                                        "cast_libraries/pyclariuscast.so")  # load the pyclariuscast.so shared library
import pyclariuscast

# Define a variable to store the received ultrasound image
processed_image = zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 4), dtype=uint8)

# Define variable for storing if image is froze
is_image_frozen = [False, False]


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

        # Initialize the ROS node for publishing the data
        init_node(CLARIUS_US_PUBLISHER)

        self.publish_node_status(INITIALIZING)
        self.log_single_message('Waiting to connect to scanner')

        # Capture current time
        init_time = Time.now()

        # Define connection success flag
        successful_connection = False

        # Define time to wait for connection
        time_to_wait = Duration(secs=5)

        while Time.now() - init_time < time_to_wait and not is_shutdown():
            # Create a casting object
            self.cast = pyclariuscast.Caster(self.new_processed_image, self.new_raw_image, self.new_spectrum_image,
                                             self.freeze_function, self.buttons_function)

            # Initialize the casting
            ret = self.cast.init(expanduser("~/"), IMAGE_WIDTH, IMAGE_HEIGHT)

            # If the initialization was successful, connect and notify the user
            if ret:
                ret = self.cast.connect(IP, PORT, "research")
                if ret:
                    successful_connection = True
                    print("Casting connected to {0} on port {1}".format(IP, PORT))
                    self.log_single_message('Connection successful')
                    break

        if not successful_connection:
            self.cast.destroy()
            self.error_message = "Casting connection to {0} on port {1} failed.".format(IP, PORT)
            self.log_single_message('Connection failed')
            # raise Exception(self.error_message)

        else:

            # Define the image publisher
            self.image_publisher = Publisher(IMAGE_SOURCE, ImageWithTimeData, queue_size=100)

            # Define a publisher for when the image has been frozen and unfrozen
            self.image_frozen_status_publisher = Publisher(IMAGE_FROZEN_STATUS, Bool, queue_size=1)

            # Define services for saving raw images
            Service(CC_SAVE_IMAGES, BoolRequest, self.save_images_handler)
            Service(CC_SAVED_IMAGES_DESTINATION, StringRequest, self.saved_images_destination_handler)

            # Define the frequency at which to publish images
            freq = 30  # CHANGE THIS LINE to change the rate at which images are published
            self.rate = Rate(freq)

            self.log_single_message('Node ready')

    # noinspection PyUnusedLocal
    @staticmethod
    def new_processed_image(image, width, height, sz, microns_per_pixel, timestamp, angle, imu):
        """Updates the new processed image variable."""
        global processed_image
        
        # bpp = sz / (width * height)
        #
        # if bpp == 4:
        #     img = Image.frombytes("RGBA", (width, height), image)
        # else:
        #     img = Image.frombytes("L", (width, height), image)

        # Process the image received
        img = QtGui.QImage(image, width, height, QtGui.QImage.Format_ARGB32)

        # Convert the image to a numpy array
        ptr = img.constBits()
        processed_image = array(ptr).reshape((height, width, 4))
        return

    @staticmethod
    def new_raw_image(image, lines, samples, bps, axial, lateral, timestamp, jpg, rf, angle):
        """Does nothing but is required for the class."""
        return

    @staticmethod
    def new_spectrum_image(image, lines, samples, bps, period, microns_per_sample, velocity_per_sample, pw):
        """Does nothing but is required for the class."""
        return

    @staticmethod
    def freeze_function(frozen):
        """Updates the global variable tracking when the image is frozen."""
        global is_image_frozen
        is_image_frozen[OLD_VALUE] = is_image_frozen[NEW_VALUE]
        is_image_frozen[NEW_VALUE] = frozen

    @staticmethod
    def buttons_function(button, clicks):
        """Updates when a button has been pressed."""
        print("button pressed: {0}, clicks: {1}".format(button, clicks))
        return

    def save_images_handler(self, req: BoolRequestRequest):
        """Handles the request to save images."""

        # Save request for future use
        self.save_incoming_images = req.value

        # If requested to save images
        if req.value:

            # If a destination for the images exists,
            if self.folder_destination is not None:

                # Create the path to the folder where the images will be saved
                self.folder_path = self.folder_destination + '/' + str(date.today()) + '_' + \
                                   datetime.now().strftime('%H-%M')

                # Make a folder if it does not already exist
                if not exists(self.folder_path):
                    makedirs(self.folder_path)
                    self.log_single_message('New folder for saved images created at ' + self.folder_path)

                # Reset the index number for each image saved and log the update
                self.saved_image_index = 1
                self.log_single_message('Image saving activated')

            # Otherwise, fail the request
            else:
                return BoolRequestResponse(False, 'No selected directory')

        # Otherwise, log the request
        else:
            self.log_single_message('Image saving de-activated')

        return BoolRequestResponse(True, NO_ERROR)

    def saved_images_destination_handler(self, req: StringRequestRequest):
        """Handle the request to update where the images should be saved."""

        # If the requested destination exists,
        if isdir(req.value):

            # Save the destination and log the change
            self.folder_destination = req.value
            self.log_single_message('New destination for saved images is ' + req.value)
            return StringRequestResponse(True, NO_ERROR)

        # Otherwise, fail the request as invalid
        else:
            return StringRequestResponse(False, 'Path is invalid')

    def save_image(self, image: array):
        """Saves the image currently saved in the class to the disk."""

        # If the image is not empty, not frozen, and the path to where to save the images exists,
        if image.sum() / (IMAGE_HEIGHT * IMAGE_WIDTH) > 25 and self.folder_path is not None:

            # Save the image to the proper location
            imsave(self.folder_path + '/Slice_' + str(self.saved_image_index).zfill(5) + FILE_EXTENSION,
                   cvtColor(image, COLOR_GRAY2BGR))

            # Increment the image counter
            self.saved_image_index = self.saved_image_index + 1

    def main_loop(self):
        while not is_shutdown():

            # Convert the image to a properly dimensioned gray scale image
            bscan = cvtColor(processed_image[:, :, 0:3], COLOR_BGR2GRAY)

            # Publish the image as an ImageWithTimeData message
            self.image_publisher.publish(convert_image_array_to_image_with_time_data_message(image_array=bscan))

            # Display the image if requested
            if VISUALIZATION_INCLUDED:
                imshow('raw image', bscan)
                waitKey(1)

            # If the image is not frozen,
            if is_image_frozen[NEW_VALUE]:

                # If the image should be saved,
                if self.save_incoming_images:

                    # Save the image and send the appropriate status
                    self.save_image(bscan)
                    self.publish_node_status(SENDING_IMAGES_SAVING_IMAGES)

                # Otherwise don't save the image and publish the appropriate status
                else:
                    self.publish_node_status(SENDING_IMAGES_NOT_SAVING_IMAGES)

            # Otherwise publish that the image is frozen
            else:
                self.publish_node_status(IMAGE_FROZEN_NOT_SAVING_IMAGES)

            # Only publish a new status when the status of the image has changed
            if is_image_frozen[NEW_VALUE] != is_image_frozen[OLD_VALUE]:
                self.image_frozen_status_publisher.publish(is_image_frozen[NEW_VALUE])

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

    node.log_single_message('Node terminating')
    print("\nShutdown signal received.")
