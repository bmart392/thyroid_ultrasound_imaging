"""
Contains user_input_polygon_points function.
"""
# Import standard packages
from copy import copy
from cv2 import imshow, waitKey, destroyWindow, line, circle, \
    setMouseCallback, EVENT_LBUTTONDOWN

# Import custom objects
from scripts.ImageData.ImageData import ImageData

# Import custom functions
from scripts.Visualizations.display_text_on_image import display_text_on_image
from scripts.Visualizations.generate_random_color import generate_random_color


def user_input_polygon_points(image_data: ImageData, polygon_use: str, list_of_points: list = None,
                              display_result: bool = False):
    """
    Prompt the user to draw a polygon by selecting points. This does not support polygons that have edges
    crossing over each other or with holes.

    Parameters
    ----------
    image_data
        the ImageData object containing the image to draw on.
    polygon_use
        the words to display to the user about the use of the polygon.
    list_of_points
        a list of points defining a polygon. If this is provided, the function does nothing
        and returns the same list of points.
    display_result
        choose to display the points selected. This can be used to capture values to hardcode.
    """
    # Do nothing if the background points have been passed in
    if list_of_points is None:
        # Define a window name to display
        window_name = 'Image Background Selection'

        # Define colors to use for displaying information
        point_color = (0, 0, 255)

        # Create a copy of the image array so that the original is not changed
        temp_image_array = copy(image_data.cropped_image)

        # Add text to the image to inform the user what needs to happen.
        temp_image_array = display_text_on_image(temp_image_array,
                                                 'Select points to define the border',
                                                 text_origin=(5, 10), text_size=0.375)
        temp_image_array = display_text_on_image(temp_image_array,
                                                 'of the ' + polygon_use + ".",
                                                 text_origin=(5, 20), text_size=0.375)
        temp_image_array = display_text_on_image(temp_image_array,
                                                 'Press any key to continue.',
                                                 text_origin=(5, 30), text_size=0.375)
        # Show the image in the properly named window
        imshow(window_name, temp_image_array)

        # Initialize the additional inputs needed for the mouseclick callback function
        list_of_points = []
        list_of_line_colors = []
        callback_inputs = [temp_image_array, point_color, window_name, list_of_line_colors, list_of_points]

        # Set the callback function
        setMouseCallback(window_name, boundary_click_event, callback_inputs)

        # Wait for the user to close the window
        waitKey(0)

        # Ensure that at least three points have been selected
        if len(list_of_points) < 3:
            raise Exception("Not enough points were selected to make a polygon." +
                            " At least 3 points must be selected.")

        # Destroy the window when the input is complete
        destroyWindow(window_name)

        # Return the list of points generated by the user
        if display_result:
            print("List of points selected for " + polygon_use + ":")
            print(list_of_points)
        return list_of_points

    else:
        # Return points passed into the function
        return list_of_points


def boundary_click_event(event: int, x, y, flags, additional_inputs: list):
    """
    Mouse-click callback function to capture a mouse click, draw a dot, and connect previously selected points
    together with a randomly colored line.

    Parameters
    ----------
    event
        the type of event that occurred which triggered the callback.
    x
        the x-position of the mouse when the event occurred.
    y
        the y-position of the mouse when the event occurred.
    flags
        an optional parameter that is not needed.
    additional_inputs
        a list declared outside this function that will be updated to include
        each new x-position and y-position selected.
    """

    # Only do something when the left mouse button has been clicked
    if event == EVENT_LBUTTONDOWN:

        # Create a new copy of the image to annotate
        temp_image_array = copy(additional_inputs[0])

        # Rename the individual additional_inputs for clarity
        list_of_points = additional_inputs[-1]
        list_of_colors = additional_inputs[-2]
        point_color = additional_inputs[1]
        window_name = additional_inputs[2]

        # Define attributes of the drawn elements
        point_radius = 3
        intermediate_line_width = 1
        final_line_color = (255, 255, 255)
        final_line_width = 2

        # Add the newly selected point to the list of existing points
        list_of_points.append((x, y))

        # Iterate through the list of points to show them on the image
        for point in list_of_points:
            temp_image_array = circle(temp_image_array, point, point_radius, point_color, -1)

        # Connect the points together with lines
        # There must be at least 2 points for this to occur
        if len(list_of_points) > 1:

            # Create a new color to use for the line and save it for future use
            list_of_colors.append(generate_random_color())

            # Iterate through the list of points to draw each line using the appropriate color
            for i in range(1, len(list_of_points)):
                temp_image_array = line(temp_image_array, list_of_points[i - 1], list_of_points[i],
                                        list_of_colors[i - 1], intermediate_line_width)

        # Connect the first and last point with a different line
        # There must be at least three points selected for this to occur
        if len(list_of_points) > 2:
            temp_image_array = line(temp_image_array, list_of_points[0], list_of_points[-1],
                                    final_line_color, final_line_width)

        # Show the updated image
        imshow(window_name, temp_image_array)
