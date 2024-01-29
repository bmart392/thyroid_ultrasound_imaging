"""
Defines the display_image_with_callback function.
"""
import _tkinter

# Import from standard packages
from matplotlib.pyplot import imshow, title, draw, figure, ginput, close
from matplotlib.figure import Figure
from matplotlib.image import AxesImage
from numpy import array, asarray, ceil, floor
from cv2 import imread

# Define rounding behavior
ROUND_STANDARD: int = int(0)
ROUND_UP: int = int(1)
ROUND_DOWN: int = int(-1)


def display_image_with_callback(image_array: array, message_to_display: str,
                                default_result: list,
                                window_name: str = "",
                                fig_to_plot_on: Figure = None,
                                axis_to_plot_on: AxesImage = None,
                                return_fig_and_axes: bool = False,
                                rounding: int = ROUND_STANDARD):
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
    fig_to_plot_on
        The figure to use to display the image.
    axis_to_plot_on
        The axis on which to plot the image.
    return_fig_and_axes
        When true, the figure and axis used to show the image are returned to the user rather than being destroyed.
    rounding
        Determines how the values selected by the user are rounded.
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

    # Capture the point selected by the user and convert the position to an integer using the proper rounding method
    try:
        selected_points = asarray(ginput(1, timeout=-1))
        if len(selected_points) == 0:
            result = selected_points
        else:
            selected_points = [j + 0.5 for j in selected_points[0]]
            if rounding == ROUND_STANDARD:
                result = [int(j) for j in selected_points]
            elif rounding == ROUND_UP:
                result = [int(ceil(j)) for j in selected_points]
                result = [int(j - 1) for j in result]
            elif rounding == ROUND_DOWN:
                result = [int(floor(j)) for j in selected_points]
            else:
                raise Exception("Rounding type of " + str(rounding) + " is not recognized.")

    # If the user closes the window
    except _tkinter.TclError:
        result = default_result

    # Ensure the result is within the bounds of the image
    if len(result) > 0 and not type(result[0]) == bool:
        if result[0] < 0:
            result[0] = 0
        elif result[0] > image_array.shape[1]:
            result[0] = image_array.shape[1]
        elif result[1] < 0:
            result[1] = 0
        elif result[1] > image_array.shape[0]:
            result[1] = image_array.shape[0]

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

    result, fig_test, axis_test = display_image_with_callback(image, "help", [1, 2], "type", return_fig_and_axes=True,
                                                              rounding=ROUND_UP)

    print(result)
    print(type(fig_test))
    print(type(axis_test))

    image = imread('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/'
                   'Test/Images/2023-11-29_19-14/Slice_00100.png')

    result = display_image_with_callback(image, "help 2", [1, 2], "type 2",
                                         fig_to_plot_on=fig_test,
                                         axis_to_plot_on=axis_test)

    print(result)
