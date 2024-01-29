"""
Contains the code for the add_centroids_on_image function.
"""

# Import standard python packages
from numpy import array
from cv2 import circle

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData


def add_centroids_on_image(image_to_show: array, image_data: ImageData,
                           dot_radius: int = 6, dot_color: tuple = (255, 0, 0)) -> array:
    """
    Draw a dot for each centroid from a given ImageData object on a given image. Returns the input image array.

    Parameters
    ----------
    image_to_show
        a numpy array representing the image to show.
    image_data
        the ImageData object that contains the centroids to show.
    dot_radius
        the size of the dot.
    dot_color
        the color of each dot.
    """
    # Draw each centroid from the image
    for centroid in image_data.contour_centroids:
        image_to_show = circle(image_to_show,
                               centroid,
                               radius=dot_radius,
                               color=dot_color,
                               thickness=-1)
    return image_to_show
