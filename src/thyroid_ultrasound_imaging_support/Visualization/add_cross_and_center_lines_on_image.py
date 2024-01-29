"""
Contains all code for add_cross_and_center_lines_on_image function.
"""

# TODO - Dream - Add a nice way to set the parameters for each sub function from this function

# Import standard python packages
from numpy import array

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.Visualization.add_centroids_on_image import add_centroids_on_image
from thyroid_ultrasound_imaging_support.Visualization.add_center_lines_to_image import add_center_lines_to_image


def add_cross_and_center_lines_on_image(image_to_show: array, image_data: ImageData):
    """
    Add both a cross and the centroids from a given ImageData object to a given image array.
    This function draws the centroids, then the cross. Returns the input image array.

    Parameters
    ----------
    image_to_show
        a numpy array representing the image to show.
    image_data
        the ImageData object that contains the centroids to show.
    """
    return add_center_lines_to_image(add_centroids_on_image(image_to_show, image_data, dot_radius=2))