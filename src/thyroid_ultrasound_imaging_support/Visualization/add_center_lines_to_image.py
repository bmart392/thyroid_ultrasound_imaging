"""
Contains code for the add_center_lines_to_image.
"""

# Import standard python packages
from numpy import array, zeros, uint8
from cv2 import line, rectangle


def add_center_lines_to_image(image_to_show: array,
                              include_vertical_line: bool = True,
                              vertical_line_location: float = 0,
                              vertical_line_color: tuple = (255, 255, 255),
                              vertical_line_thickness: int = 1,
                              vertical_line_error: int = None,
                              vertical_line_error_color: tuple = (0, 25, 0),
                              include_horizontal_line: bool = False,
                              horizontal_line_location: float = 0,
                              horizontal_line_color: tuple = (255, 255, 255),
                              horizontal_line_error: int = None,
                              horizontal_line_error_color: tuple = (0, 25, 0),
                              horizontal_line_thickness: int = None,
                              ):
    """
    Add two lines to a given image to form a cross centered on the image. Returns the input image array.

    Parameters
    ----------
    horizontal_line_thickness :
        the thickness of the lines drawn.
    horizontal_line_error_color :
    horizontal_line_error :
    horizontal_line_location :
    vertical_line_error_color :
    vertical_line_error :
    vertical_line_thickness :
        the thickness of the lines drawn.
    vertical_line_location :
    image_to_show
        a numpy array representing the image to show.
    include_vertical_line
        a flag to indicate if a vertical line should be drawn.
    include_horizontal_line
        a flag to indicate if a vertical line should be drawn.
    vertical_line_color
        the color of the vertical line.
    horizontal_line_color
        the color of the horizontal line.
    """

    # Draw vertical line
    if include_vertical_line:

        vertical_line_x = int((image_to_show.shape[1] / 2) + (image_to_show.shape[1] * vertical_line_location)) - 1

        if vertical_line_error is not None:
            error_region = rectangle(zeros(image_to_show.shape, dtype=uint8),
                                     (vertical_line_x - vertical_line_error, 0),
                                     (vertical_line_x + vertical_line_error, image_to_show.shape[0] - 1),
                                     color=vertical_line_error_color,
                                     thickness=-1)
            image_to_show = image_to_show + error_region

        image_to_show = line(image_to_show,
                             (vertical_line_x, 0),
                             (vertical_line_x, image_to_show.shape[0] - 1),
                             color=vertical_line_color, thickness=vertical_line_thickness)

    # Draw horizontal line
    if include_horizontal_line:

        if horizontal_line_thickness is None:
            horizontal_line_thickness = vertical_line_thickness

        horizontal_line_y = int((image_to_show.shape[0] / 2) + (image_to_show.shape[0] * horizontal_line_location)) - 1
        if horizontal_line_error is not None:
            error_region = rectangle(zeros(image_to_show.shape, dtype=uint8),
                                     (0, horizontal_line_y - horizontal_line_error),
                                     (image_to_show.shape[1] - 1, horizontal_line_y + horizontal_line_error),
                                     color=horizontal_line_error_color,
                                     thickness=-1)
            image_to_show = image_to_show + error_region
        image_to_show = line(image_to_show,
                             (0, horizontal_line_y),
                             (image_to_show.shape[1] - 1, horizontal_line_y),
                             color=horizontal_line_color, thickness=horizontal_line_thickness)

    return image_to_show
