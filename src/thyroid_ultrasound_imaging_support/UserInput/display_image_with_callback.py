"""
Defines the display_image_with_callback function.
"""
import _tkinter

# Import from standard packages
from matplotlib.pyplot import imshow, title, draw, figure, ginput, close
from numpy import array, asarray
from cv2 import imread


def display_image_with_callback(image_array: array, message_to_display: str,
                                default_result: list,
                                window_name: str = ""):
    """
    Display an image and capture a point selected by the user.

    Parameters
    ----------
    image_array
        The array representing the image to display.
    message_to_display
        The message to display to the user above the image.
    default_result
        The result to return if no point is selected by the user.
    window_name
        The name to display as the window name.
    """

    # Create a new figure
    figure(window_name, figsize=(6, 6), dpi=300)

    # Display the image in the plot
    imshow(image_array)

    # Display a message to the user
    title(message_to_display, fontsize=8)
    draw()

    # Capture the point selected by the user and convert the position to an integer
    try:
        selected_points = asarray(ginput(1, timeout=-1))
        if len(selected_points) == 0:
            result = selected_points
        else:
            result = [int(j) for j in selected_points[0]]

    # If the user closes the window
    except _tkinter.TclError:
        result = default_result

    close()

    return result


if __name__ == '__main__':
    image = imread('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/Test/Images/Series2/Slice_30.png')

    print(display_image_with_callback(image, "help", [1, 2], "type"))
