"""
Defines the display_image_with_callback function.
"""
import _tkinter

# Import from standard packages
from matplotlib.pyplot import imshow, title, draw, figure, ginput, close
from matplotlib.figure import Figure
from matplotlib.image import AxesImage
from numpy import array, asarray
from cv2 import imread


def display_image_with_callback(image_array: array, message_to_display: str,
                                default_result: list,
                                window_name: str = "",
                                fig_to_plot_on: Figure = None,
                                axis_to_plot_on: AxesImage = None,
                                return_fig_and_axes: bool = False):
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
    return_fig_and_axes
        When true, the figure and axis used to show the image are returned to the user rather than being destroyed.
    """

    # Try to use the figure and axes passed in to plot the image
    try:
        fig = fig_to_plot_on
        fig.canvas.set_window_title(window_name)
        axis = axis_to_plot_on
        axis.set_data(image_array)

    # If either do not exist, create a new figure and axis
    except AttributeError:
        fig = figure(window_name, figsize=(6, 6), dpi=300)
        axis = imshow(image_array)

    # Display a message to the user as the title of the window
    title(message_to_display, fontsize=8)

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

    # Return the figure and axis, if requested by the user
    if return_fig_and_axes:
        return result, fig, axis

    # Otherwise,
    else:

        # Close the window
        close()

        # Return only the result
        return result


if __name__ == '__main__':
    image = imread('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/'
                   'Test/Images/2023-11-29_19-14/Slice_00002.png')

    result, fig_test, axis_test = display_image_with_callback(image, "help", [1, 2], "type", return_fig_and_axes=True)

    print(result)
    print(type(fig_test))
    print(type(axis_test))

    image = imread('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/'
                   'Test/Images/2023-11-29_19-14/Slice_00100.png')

    result = display_image_with_callback(image, "help 2", [1, 2], "type 2",
                                         fig_to_plot_on=fig_test,
                                         axis_to_plot_on=axis_test)

    print(result)
