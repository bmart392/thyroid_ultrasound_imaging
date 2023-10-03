"""
Define object class for parent of all filter types.
"""

# Import standard packages
from time import time
import numpy as np
import cv2
from copy import copy

# Import custom objects
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData

# Import custom constants
from thyroid_ultrasound_imaging_support.ImageFilter.FilterConstants import *

# Import custom functions
from thyroid_ultrasound_imaging_support.Visualization.Visualization import Visualization
from thyroid_ultrasound_imaging_support.Visualization.display_process_timer import display_process_timer
from thyroid_ultrasound_imaging_support.UserInput.user_input_crop_coordinates import user_input_crop_coordinates


class ImageFilter:
    """
    A parent class for all image filters to extend. Defines basic function calls
    """

    def __init__(self, image_crop_included: bool = True, image_crop_coordinates: list = None):
        """
        Initialization function for an ImageFilter object.

        Parameters
        ----------
        image_crop_included
            a flag to indicate if the image needs to be cropped.

        image_crop_coordinates
            an iterable containing two points defining the rectangle with which to crop the image.
        """
        self.image_crop_included: bool = image_crop_included
        self.image_crop_coordinates: np.array = image_crop_coordinates
        self.filter_color = None
        self.previous_image_mask_array = None
        self.debug_mode: bool = False
        self.analysis_mode: bool = False
        self.increase_contrast: bool = False

    # --------------------------------------
    # Start image filtering function section
    # --------------------------------------

    def fully_filter_image(self, image_data: ImageData):
        """
        Fully filter an ImageData object using the process defined in the ImageFilter

        Parameters
        ----------
        image_data
            the ImageData object containing the image to be filtered.
        """

        # record the current time for timing each process
        start_of_process_time = time()

        # Crop the image
        self.crop_image(image_data)
        start_of_process_time = self.display_process_timer(start_of_process_time,
                                                           "Crop image time")

        # Colorize the image
        self.colorize_image(image_data)
        start_of_process_time = self.display_process_timer(start_of_process_time,
                                                           "Recolor image time")

        # Pre-process the image
        self.pre_process_image(image_data)
        start_of_process_time = self.display_process_timer(start_of_process_time,
                                                           "Pre-process image time")

        # Create image mask
        self.create_image_mask(image_data)
        start_of_process_time = self.display_process_timer(start_of_process_time,
                                                           "Mask creation time")

        # Post-process the image mask
        self.image_mask_post_process(image_data)
        start_of_process_time = self.display_process_timer(start_of_process_time,
                                                           "Post-process image mask time")

        # Expand the image mask if the image was cropped
        self.expand_image_mask(image_data)
        start_of_process_time = self.display_process_timer(start_of_process_time,
                                                           "Mask expansion time")

        # Create sure foreground mask
        self.create_sure_foreground_mask(image_data)
        start_of_process_time = self.display_process_timer(start_of_process_time,
                                                           "Sure foreground mask creation time")

        # Create sure background mask
        self.create_sure_background_mask(image_data)
        start_of_process_time = self.display_process_timer(start_of_process_time,
                                                           "Sure background mask creation time")

        # Create probable foreground mask
        self.create_probable_foreground_mask(image_data)
        self.display_process_timer(start_of_process_time,
                                   "Probable foreground mask creation time")

        # Set the image mask to use for the next iteration
        self.previous_image_mask_array = (np.zeros(image_data.original_image.shape[:2], np.uint8) +
                                          image_data.sure_foreground_mask * cv2.GC_FGD +
                                          image_data.sure_background_mask * cv2.GC_BGD +
                                          image_data.probable_foreground_mask * cv2.GC_PR_FGD)
        if self.image_crop_included:
            crop_x_0 = self.image_crop_coordinates[0][0]
            crop_y_0 = self.image_crop_coordinates[0][1]
            crop_x_1 = self.image_crop_coordinates[1][0]
            crop_y_1 = self.image_crop_coordinates[1][1]
            self.previous_image_mask_array = self.previous_image_mask_array[crop_y_0:crop_y_1, crop_x_0:crop_x_1]

    def pre_process_image(self, image_data: ImageData):
        raise Exception("This function was not implemented in the sub-class.")

    def create_image_mask(self, image_data: ImageData):
        raise Exception("This function was not implemented in the sub-class.")

    def image_mask_post_process(self, image_data: ImageData):
        raise Exception("This function was not implemented in the sub-class.")

    @staticmethod
    def create_sure_foreground_mask(image_data: ImageData):
        raise Exception("This function was not implemented in this sub-class.")

    @staticmethod
    def create_sure_background_mask(image_data: ImageData):
        raise Exception("This function was not implemented in this sub-class.")

    @staticmethod
    def create_probable_foreground_mask(image_data: ImageData):
        raise Exception("This function was not implemented in this sub-class.")

    # -----------------------------
    # Start helper function section
    # -----------------------------

    def colorize_image(self, image_data: ImageData) -> ImageData:
        """
            Recolor the image based on the filter color of the image filter.

            Parameters
            ----------
            image_data: ImageData
                the image data object containing the image to be recolored.
        """

        # recolor the image if the image is not in the desired color for the filter
        if not self.filter_color == image_data.image_color:

            # check for each combination of color recoloring and assign the correct constant

            if ((self.filter_color == COLOR_BGR or self.filter_color == COLOR_RGB) and
                    image_data.image_color == COLOR_GRAY):
                new_color = cv2.COLOR_GRAY2BGR  # Equivalent to cv2.COLOR_GRAYRGB

            elif ((self.filter_color == COLOR_BGR or self.filter_color == COLOR_RGB) and
                  (image_data.image_color == COLOR_BGR or image_data.image_color == COLOR_RGB)):
                new_color = cv2.COLOR_BGR2RGB  # Equivalent to cv2.COLOR_RGB2BGR

            elif self.filter_color == COLOR_GRAY and image_data.image_color == COLOR_BGR:
                new_color = cv2.COLOR_BGR2GRAY

            elif self.filter_color == COLOR_GRAY and image_data.image_color == COLOR_RGB:
                new_color = cv2.COLOR_RGB2GRAY

            else:
                raise Exception("Filter color and image color combination not included.")

            # set the colorized image based on the color required by the filter
            image_data.colorized_image = cv2.cvtColor(image_data.cropped_image, new_color)

        # otherwise set the colorized image as the original image
        else:
            image_data.colorized_image = image_data.cropped_image

        # return the image data object
        return image_data

    def crop_image(self, image_data: ImageData) -> ImageData:
        """
            Crop the original image, if included in the filter, using the crop coordinates stored in the filter.

            Parameters
            ----------
            image_data: ImageData
                the image data object containing the image to be cropped.
        """

        # If the image needs to be cropped
        if self.image_crop_included:

            # Define the x and y values of the crop region
            crop_x_0 = self.image_crop_coordinates[0][0]
            crop_y_0 = self.image_crop_coordinates[0][1]
            crop_x_1 = self.image_crop_coordinates[1][0]
            crop_y_1 = self.image_crop_coordinates[1][1]

            # check that the image has the correct shape, then crop it
            if image_data.original_image.ndim > 3 or image_data.original_image.ndim < 2:
                raise Exception("The dimensions of the original image are invalid.")
            elif image_data.original_image.ndim == 3:
                image_data.cropped_image = copy(image_data.original_image[crop_y_0:crop_y_1, crop_x_0:crop_x_1, :])
            else:
                image_data.cropped_image = copy(image_data.original_image[crop_y_0:crop_y_1, crop_x_0:crop_x_1])

        # Otherwise pass the original image on
        else:
            image_data.cropped_image = copy(image_data.original_image)

    def expand_image_mask(self, image_data: ImageData) -> ImageData:
        """
            Expand the image mask generated by the filter to the full size of the image, if necessary.

            Parameters
            ----------
            image_data: ImageData
                the image data object containing the image to be expanded.
        """

        # Expand the mask if it is not the same size as the original image
        if not image_data.original_image.shape[:2] == image_data.image_mask.shape:

            # Create an empty image mask
            image_data.expanded_image_mask = np.zeros(image_data.original_image.shape[:2], np.uint8)

            # Copy the smaller mask data on to the original data
            for row in range(image_data.image_mask.shape[0]):
                for column in range(image_data.image_mask.shape[1]):
                    image_data.expanded_image_mask[row + self.image_crop_coordinates[0][1],
                                                   column + self.image_crop_coordinates[0][0]] = copy(
                        image_data.image_mask[row, column]
                    )

        # Else copy the image mask to the expanded mask
        else:
            image_data.expanded_image_mask = image_data.image_mask

    def display_process_timer(self, start_of_process_time, message) -> float:
        """
        Display the amount of time a process ran in milliseconds. Returns the current time of the function call.

        Parameters
        ----------
        start_of_process_time : float
            Time the process being measured started as marked with a call to time().

        message: str
            Description to display with the measured time.
        """
        return display_process_timer(start_of_process_time, message, self.analysis_mode)

    def generate_crop_coordinates(self, image_data: ImageData):
        """
        Prompt the user to crop the image correctly. Then crop the passed in image.

        Parameters
        ----------
        image_data
            The ImageData object containing the image to be cropped.
        """
        self.image_crop_coordinates = user_input_crop_coordinates(image_data)
        self.image_crop_included = True
        self.crop_image(image_data)

