"""
Contains import statements and constants used in image filter objects.
"""

# Import standard packages
from time import time
import numpy as np
import cv2
from copy import copy
from skimage.restoration import denoise_nl_means, estimate_sigma

# Import custom objects
from scripts.ImageData.ImageData import ImageData

# Define constants to be used by any image filter implementing this interface
COLOR_BGR: int = 0
COLOR_RGB: int = 1
COLOR_GRAY: int = 2

NO_BLUR: int = 0
BLUR_GAUSSIAN: int = 1
BLUR_MEAN_FILTER: int = 2

THRESHOLD_BASIC: int = 0
THRESHOLD_ADAPTIVE: int = 1

MASK_ERODE: int = 0
MASK_DILATE: int = 1
MASK_OPEN: int = 2
MASK_CLOSE: int = 3
MASK_RECT: int = 4


# --------------------- Define custom functions -------------------------


def colorize_image(filter_color: int, image_data: ImageData) -> ImageData:
    """
        Recolor the image based on the filter color of the image filter.

        Parameters
        ----------
        filter_color: int
            the image color the filter requires.
        image_data: ImageData
            the image data object containing the image to be recolored.
    """

    # recolor the image if the image is not in the desired color for the filter
    if not filter_color == image_data.image_color:

        # check for each combination of color recoloring and assign the correct constant

        if ((filter_color == COLOR_BGR or filter_color == COLOR_RGB) and
                image_data.image_color == COLOR_GRAY):
            new_color = cv2.COLOR_GRAY2BGR  # Equivalent to cv2.COLOR_GRAYRGB

        elif ((filter_color == COLOR_BGR or filter_color == COLOR_RGB) and
              (image_data.image_color == COLOR_BGR or image_data.image_color == COLOR_RGB)):
            new_color = cv2.COLOR_BGR2RGB  # Equivalent to cv2.COLOR_RGB2BGR

        elif filter_color == COLOR_GRAY and image_data.image_color == COLOR_BGR:
            new_color = cv2.COLOR_BGR2GRAY

        elif filter_color == COLOR_GRAY and image_data.image_color == COLOR_RGB:
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


def crop_image(image_crop_included: bool, image_crop_coordinates: list, image_data: ImageData) -> ImageData:
    """
        Crop the original image based on the needs of the color filter.

        Parameters
        ----------
        image_crop_included: bool
            flag indicating if a cropping operation is required on the image
        image_crop_coordinates: list
            two xy coordinate pairs indicating which region of the image to keep
        image_data: ImageData
            the image data object containing the image to be cropped.
    """

    # If the image needs to be cropped
    if image_crop_included:

        # Define the x and y values of the crop region
        crop_x_0 = image_crop_coordinates[0][0]
        crop_y_0 = image_crop_coordinates[0][1]
        crop_x_1 = image_crop_coordinates[1][0]
        crop_y_1 = image_crop_coordinates[1][1]

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

    # Return the modified image_data
    return image_data


def expand_image_mask(image_crop_coordinates: list, image_data: ImageData) -> ImageData:
    """
        Expand the image mask generated by the filter to the full size of the image, if necessary.

        Parameters
        ----------
        image_crop_coordinates: list
            two xy coordinate pairs indicating which region of the image was kept in the crop operation.
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
                image_data.expanded_image_mask[row + image_crop_coordinates[0][1],
                                               column + image_crop_coordinates[0][0]] = copy(
                    image_data.image_mask[row, column]
                )

    # Else copy the image mask to the expanded mask
    else:
        image_data.expanded_image_mask = image_data.image_mask

    # Return the updated image data object
    return image_data


def display_process_timer(start_of_process_time, message, print_time=True) -> float:
    """
    Display the amount of time a process ran in milliseconds.

    Parameters
    ----------
    start_of_process_time : float
        Time the process being measured started as marked with a call to time().

    message: str
        Description to display with the measured time.

    print_time: Bool
        Selector for printing the time to allow for easier debugging.
    """
    if print_time:
        print(message + " (ms): ", round((time() - start_of_process_time) * 1000, 4))
    return time()


def fully_filter_image(image_filter, image_data: ImageData):
    # record the current time for timing each process
    start_of_process_time = time()

    # Crop the image
    image_data = crop_image(image_filter.image_crop_included,
                            image_filter.image_crop_coordinates,
                            image_data)
    start_of_process_time = display_process_timer(start_of_process_time,
                                                  "Crop image time",
                                                  image_filter.analysis_mode)

    # Colorize the image
    image_data = colorize_image(image_filter.filter_color, image_data)
    start_of_process_time = display_process_timer(start_of_process_time,
                                                  "Recolor image time",
                                                  image_filter.analysis_mode)

    # Pre-process the image
    image_data = image_filter.pre_process_image(image_data)
    start_of_process_time = display_process_timer(start_of_process_time,
                                                  "Pre-process image time",
                                                  image_filter.analysis_mode)

    # Create image mask
    image_data = image_filter.create_image_mask(image_data)
    start_of_process_time = display_process_timer(start_of_process_time,
                                                  "Mask creation time",
                                                  image_filter.analysis_mode)

    # Post-process the image mask
    image_data = image_filter.image_mask_post_process(image_data)
    start_of_process_time = display_process_timer(start_of_process_time,
                                                  "Post-process image mask time",
                                                  image_filter.analysis_mode)

    # Expand the image mask if the image was cropped
    image_data = expand_image_mask(image_filter.image_crop_coordinates, image_data)
    start_of_process_time = display_process_timer(start_of_process_time,
                                                  "Mask expansion time",
                                                  image_filter.analysis_mode)

    # Create sure foreground mask
    image_data = image_filter.create_sure_foreground_mask(image_data)
    start_of_process_time = display_process_timer(start_of_process_time,
                                                  "Sure foreground mask creation time",
                                                  image_filter.analysis_mode)

    # Create sure background mask
    image_data = image_filter.create_sure_background_mask(image_data)
    start_of_process_time = display_process_timer(start_of_process_time,
                                                  "Sure background mask creation time",
                                                  image_filter.analysis_mode)

    # Create probable foreground mask
    image_data = image_filter.create_probable_foreground_mask(image_data)
    display_process_timer(start_of_process_time,
                          "Probable foreground mask creation time",
                          image_filter.analysis_mode)

    # Set the image mask to use for the next iteration
    image_filter.previous_image_mask_array = (np.zeros(image_data.original_image.shape[:2], np.uint8) +
                                              image_data.sure_foreground_mask * cv2.GC_FGD +
                                              image_data.sure_background_mask * cv2.GC_BGD +
                                              image_data.probable_foreground_mask * cv2.GC_PR_FGD)
    if image_filter.image_crop_included:
        crop_x_0 = image_filter.image_crop_coordinates[0][0]
        crop_y_0 = image_filter.image_crop_coordinates[0][1]
        crop_x_1 = image_filter.image_crop_coordinates[1][0]
        crop_y_1 = image_filter.image_crop_coordinates[1][1]
        image_filter.previous_image_mask_array = image_filter.previous_image_mask_array[crop_y_0:crop_y_1, crop_x_0:crop_x_1]

    return image_data
