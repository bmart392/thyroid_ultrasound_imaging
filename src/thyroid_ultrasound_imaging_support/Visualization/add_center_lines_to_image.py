"""
Contains code for the add_center_lines_to_image.
"""

# Import standard python packages
from numpy import array
from cv2 import line


def add_center_lines_to_image(image_to_show: array,
                              include_vertical_line: bool = True,
                              include_horizontal_line: bool = False,
                              vertical_line_color: tuple = (255, 255, 255),
                              horizontal_line_color: tuple = (255, 255, 255), line_thickness: int = 1):
    """
    Add two lines to a given image to form a cross centered on the image. Returns the input image array.

    Parameters
    ----------
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
    line_thickness
        the thickness of the lines drawn.
    """

    # Draw vertical line
    if include_vertical_line:
        image_to_show = line(image_to_show,
                             (int(image_to_show.shape[1] / 2) - 1, 0),
                             (int(image_to_show.shape[1] / 2) - 1, image_to_show.shape[0] - 1),
                             color=vertical_line_color, thickness=line_thickness)

    # Draw horizontal line
    if include_horizontal_line:
        image_to_show = line(image_to_show,
                             (0, int(image_to_show.shape[0] / 2) - 1),
                             (image_to_show.shape[1] - 1, int(image_to_show.shape[0] / 2) - 1),
                             color=horizontal_line_color, thickness=line_thickness)

    return image_to_show
