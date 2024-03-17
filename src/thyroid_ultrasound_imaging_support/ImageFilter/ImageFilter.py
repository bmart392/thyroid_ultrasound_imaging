"""
Define object class for parent of all filter types.
"""

# TODO - Dream - Add an un-crop feature that adds zeros to an array to get back to the starting size
# TODO - Dream - Properly call out all of my exceptions and remove all bare "except"

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
from thyroid_ultrasound_imaging_support.ImageFilter.SegmentationError import *

# Define constants for down-sampling related tasks
DOWN_SAMPLING_MODE: int = cv2.INTER_LINEAR
UP_SAMPLING_MODE: int = cv2.INTER_CUBIC
RESIZE_SHAPE: tuple = (0, 0)


class ImageFilter:
    """
    A parent class for all image filters to extend. Defines basic function calls
    """

    def __init__(self, image_crop_coordinates: list = None):
        """
        Initialization function for an ImageFilter object.

        Parameters
        ----------
        image_crop_coordinates
            an iterable containing two points defining the rectangle with which to crop the image.
        """
        self.image_crop_coordinates: np.array = image_crop_coordinates
        self.ready_to_filter = False
        self.filter_color = None
        self.previous_image_mask_array = None
        self.debug_mode: bool = False
        self.analysis_mode: bool = False
        self.increase_contrast: bool = False
        self.is_in_contact_with_patient: bool = False
        self.down_sampling_rate: float = 0.5
        self.up_sampling_rate: float = 1 / self.down_sampling_rate

    ###########################
    # Image filtering functions
    # region

    def filter_image(self, image_data: ImageData):
        """
        Filters an ImageData object using the process defined in the ImageFilter.

        Parameters
        ----------
        image_data
            the ImageData object containing the image to be filtered.
        """
        # Perform the basic image filtering actions
        self.basic_filter_image(image_data)

    def basic_filter_image(self, image_data: ImageData):
        """
        Perform the basic image filtering operations on the ImageData object using the process defined in the
        ImageFilter

        Parameters
        ----------
        image_data
            the ImageData object containing the image to be filtered.
        """

        # record the current time for timing each process
        start_of_process_time = time()

        try:
            # Crop the image
            self.crop_image(image_data)
            start_of_process_time = self.display_process_timer(start_of_process_time,
                                                               "Crop image time")
        except Exception as caught_exception:
            raise SegmentationError(CROP_FAILURE + " in 'basic_image_filter' in ImageFilter.py",
                                    caught_exception.args[0])

        try:
            # Colorize the image
            self.colorize_image(image_data)
            start_of_process_time = self.display_process_timer(start_of_process_time,
                                                               "Recolor image time")
        except Exception as caught_exception:
            raise SegmentationError(COLORIZE_FAILURE + " in 'basic_image_filter' in ImageFilter.py",
                                    caught_exception.args[0])

        try:
            # Down-sample the image
            self.down_sample_image(image_data)
            start_of_process_time = self.display_process_timer(start_of_process_time,
                                                               "Down-sample time")
        except Exception as caught_exception:
            raise SegmentationError(DOWN_SAMPLE_FAILURE + " in 'basic_image_filter' in ImageFilter.py",
                                    caught_exception.args[0])

        try:
            # Pre-process the image
            self.pre_process_image(image_data)
            start_of_process_time = self.display_process_timer(start_of_process_time,
                                                               "Pre-process image time")
        except Exception as caught_exception:
            raise SegmentationError(PRE_PROCESS_FAILURE + " in 'basic_image_filter' in ImageFilter.py",
                                    caught_exception.args[0])

        try:
            # Create image mask
            self.create_image_mask(image_data)
            start_of_process_time = self.display_process_timer(start_of_process_time,
                                                               "Mask creation time")
        except Exception as caught_exception:
            raise SegmentationError(CREATE_MASK_FAILURE + " in 'basic_image_filter' in ImageFilter.py",
                                    caught_exception.args[0])

        try:
            # Post-process the image mask
            self.post_process_image_mask(image_data)
            start_of_process_time = self.display_process_timer(start_of_process_time,
                                                               "Post-process image mask time")
        except Exception as caught_exception:
            raise SegmentationError(POST_PROCESS_FAILURE + " in 'basic_image_filter' in ImageFilter.py",
                                    caught_exception.args[0])

        try:
            # Create sure foreground mask
            self.create_sure_foreground_mask(image_data)
            start_of_process_time = self.display_process_timer(start_of_process_time,
                                                               "Sure foreground mask creation time")
        except Exception as caught_exception:
            raise SegmentationError(CREATE_SURE_FOREGROUND_FAILURE + " in 'basic_image_filter' in ImageFilter.py",
                                    caught_exception.args[0])

        try:
            # Create sure background mask
            self.create_sure_background_mask(image_data)
            start_of_process_time = self.display_process_timer(start_of_process_time,
                                                               "Sure background mask creation time")
        except Exception as caught_exception:
            raise SegmentationError(CREATE_SURE_BACKGROUND_FAILURE + " in 'basic_image_filter' in ImageFilter.py",
                                    caught_exception.args[0])

        try:
            # Create probable foreground mask
            self.create_probable_foreground_mask(image_data)
            start_of_process_time = self.display_process_timer(start_of_process_time,
                                                               "Probable foreground mask creation time")
        except Exception as caught_exception:
            raise SegmentationError(CREATE_PROBABLE_FOREGROUND_FAILURE + " in 'basic_image_filter' in ImageFilter.py",
                                    caught_exception.args[0])

    def pre_process_image(self, image_data: ImageData):
        raise Exception("This function was not implemented in the sub-class.")

    def create_image_mask(self, image_data: ImageData):
        raise Exception("This function was not implemented in the sub-class.")

    def post_process_image_mask(self, image_data: ImageData):
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

    # endregion
    ###########################

    ##################
    # Helper Functions
    # region

    def crop_image(self, image_data: ImageData):
        """
            Crop the original image, if included in the filter, using the crop coordinates stored in the filter.
            For this crop function, the pixels selected as the crop coordinates are retained in the cropped image.

            Parameters
            ----------
            image_data: ImageData
                the image data object containing the image to be cropped.
        """

        # If the image does not already have crop coordinates
        if self.image_crop_coordinates is None:

            # Set them as the full image
            self.image_crop_coordinates = [[int(0), int(0)],
                                           [int(image_data.image_size_x) - 1, int(image_data.image_size_y) - 1]]

            # Save the crop coordinates in the image data
            image_data.image_crop_coordinates = copy(self.image_crop_coordinates)

            # Save the original image as the cropped image
            image_data.cropped_image = copy(image_data.original_image)

        # Otherwise try to crop the image
        else:

            try:
                # Define the x and y values of the crop region
                crop_x_0 = self.image_crop_coordinates[0][0]
                crop_y_0 = self.image_crop_coordinates[0][1]
                crop_x_1 = self.image_crop_coordinates[1][0]
                crop_y_1 = self.image_crop_coordinates[1][1]
            except IndexError:
                raise SegmentationError("The image crop coordinates, " + str(self.image_crop_coordinates) +
                                        " do not have the expected shape of 2x2.")

            # Check that the image has the correct shape
            if image_data.original_image.ndim > 3 or image_data.original_image.ndim < 2:
                raise SegmentationError("The dimensions of the original image are invalid")

            # Check that the 1st crop coordinate is within the bounds of the image
            elif crop_x_0 < 0 or crop_y_0 < 0:
                raise SegmentationError("The top left crop coordinate, " + str((crop_x_0, crop_y_0)) +
                                        ", is not within the bounds of the image")
            # Check that the 2nd crop coordinate is within the bounds of the image
            elif crop_x_1 > image_data.image_size_x - 1 or crop_y_1 > image_data.image_size_y - 1:
                raise SegmentationError("The bottom right crop coordinate, " + str((crop_x_1, crop_y_1)) +
                                        ", is not within the bounds of the image")

            # Check that the 2nd crop coordinate is below and to the right of the first coordinate
            elif crop_x_0 >= crop_x_1 or crop_y_0 >= crop_y_1:
                raise SegmentationError("The top left coordinate, " + str((crop_x_0, crop_y_0)) +
                                        ", is not above and to the left of the bottom right coordinate, " +
                                        str((crop_x_1, crop_y_1)))

            # Then crop the image
            elif image_data.original_image.ndim == 3:
                image_data.cropped_image = copy(
                    image_data.original_image[crop_y_0:crop_y_1 + 1, crop_x_0:crop_x_1 + 1, :])
            else:
                image_data.cropped_image = copy(image_data.original_image[crop_y_0:crop_y_1 + 1, crop_x_0:crop_x_1 + 1])

            # Save the crop coordinates in the image data object
            image_data.image_crop_coordinates = [(int(crop_x_0), int(crop_y_0)), (int(crop_x_1), int(crop_y_1))]

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

    def down_sample_image(self, image_data: ImageData):
        """
        Down-sample the colorized image using the down-sampling rate of the filter.

        Parameters
        ----------
        image_data
            The image data object containing the colorized image to down-sample.
        """
        image_data.down_sampled_image = cv2.resize(image_data.colorized_image, RESIZE_SHAPE,
                                                   fx=self.down_sampling_rate,
                                                   fy=self.down_sampling_rate,
                                                   interpolation=DOWN_SAMPLING_MODE)

        # Save the down-sampling data
        image_data.down_sampling_rate = self.down_sampling_rate
        image_data.ds_image_size_x = image_data.down_sampled_image.shape[1]
        image_data.ds_image_size_y = image_data.down_sampled_image.shape[0]

    # def expand_image_mask(self, image_data: ImageData):
    #     """
    #         Expand the image mask generated by the filter to the full size of the image, if necessary.
    #
    #         Parameters
    #         ----------
    #         image_data: ImageData
    #             the image data object containing the image to be expanded.
    #     """
    #     # Up-sample the image mask
    #     image_data.expanded_image_mask = cv2.resize(image_data.image_mask, (0, 0), fx=self.down_sampling_rate,
    #                                                 fy=self.down_sampling_rate, interpolation=cv2.INTER_CUBIC)
    #
    #     # Expand the mask if it is not the same size as the original image
    #     if not image_data.original_image.shape[:2] == image_data.expanded_image_mask.shape:
    #
    #         # Calculate the number of zeros to pad in each dimension
    #         padding = [(self.image_crop_coordinates[0][1], self.image_crop_coordinates[1][1])]
    #
    #         # Create an empty image mask
    #         image_data.expanded_image_mask = np.zeros(image_data.original_image.shape[:2], np.uint8)
    #
    #         # Copy the smaller mask data on to the original data
    #         for row in range(image_data.image_mask.shape[0]):
    #             for column in range(image_data.image_mask.shape[1]):
    #                 image_data.expanded_image_mask[row + self.image_crop_coordinates[0][1],
    #                                                column + self.image_crop_coordinates[0][0]] = copy(
    #                     image_data.image_mask[row, column]
    #                 )
    #
    #     # Else copy the image mask to the expanded mask
    #     else:
    #         image_data.expanded_image_mask = image_data.image_mask

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

    # def generate_crop_coordinates(self, image_data: ImageData):
    #     """
    #     Prompt the user to crop the image correctly. Then crop the passed in image.
    #
    #     Parameters
    #     ----------
    #     image_data
    #         The ImageData object containing the image to be cropped.
    #     """
    #     self.image_crop_coordinates = user_input_crop_coordinates(image_data)
    #     self.image_crop_included = True
    #     self.crop_image(image_data)

    # endregion
    ##################
