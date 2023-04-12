"""
Contains display_text_on_image_function.
"""
# Import standard packages
from numpy import array
from cv2 import FONT_HERSHEY_SIMPLEX, putText


def display_text_on_image(image: array, text: str, text_origin: tuple = (15, 35), text_color: tuple = (128, 0, 128),
                          text_size: float = 0.5):
    """
    Use the cv2 built-in method to add text to an image.

    Parameters
    ----------
    image
        the image array to be modified.
    text
        the message to be added to the image.
    text_origin
        the (x, y) coordinate of the top left corner of the text.
    text_color
        the color to display the text in.
    text_size
        the font size to use to display the text. This does not correlate to standard font sizes.
    """
    return putText(image, text, text_origin, FONT_HERSHEY_SIMPLEX, text_size, text_color, thickness=1,
                   bottomLeftOrigin=False)
