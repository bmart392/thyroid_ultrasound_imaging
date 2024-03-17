"""
Contains user_input_polygon_points function.
"""
# Import standard packages
from copy import copy
from cv2 import line, circle, imread, cvtColor, COLOR_GRAY2BGR, COLOR_BGR2GRAY

# Import custom objects
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData

# Import custom functions
from thyroid_ultrasound_imaging_support.UserInput.display_image_with_callback import display_image_with_callback


def user_input_polygon_points(image_data: ImageData, polygon_use: str,
                              list_of_points: list = None,
                              display_result: bool = False,
                              window_name: str = 'Image Segmentation Initialization'):
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
    window_name
        the text to display as the name of the window.
    """

    # Do nothing if the background points have been passed in
    if list_of_points is None:

        # startWindowThread()

        # Define a window name to display
        window_name = window_name

        # Define colors to use for displaying information
        point_color = (0, 255, 0)

        # Define attributes of the drawn elements
        point_radius = 4
        intermediate_line_color = (0, 0, 255)
        intermediate_line_width = 2
        final_line_color = (255, 255, 255)
        final_line_width = 2

        # Create a copy of the image array so that the original is not changed
        temp_image_array = copy(image_data.colorized_image)

        # Initialize the additional inputs needed for the mouseclick callback function
        list_of_points = []
        list_of_line_colors = []

        # Define the default result for the function to return when the window is closed
        default_result = [False]

        # Define empty variables to use to store the figure and axis used to plot each image
        fig = None
        axis = None

        # Loop until the user selects enough points
        while True:

            # If less than 3 points have been selected
            if len(list_of_points) < 3:

                # Title the image accordingly
                image_title = ("Select at least 3 points to define the border" +
                               " of the " + polygon_use + "." +
                               f"\n{len(list_of_points)} points currently selected." +
                               "\nPress the middle mouse button to restart." +
                               "\nClose the window to finish selecting points.")
            else:
                # Title the image accordingly
                image_title = ("Continue selecting points to define the border" +
                               " of the " + polygon_use + "." +
                               f"\n{len(list_of_points)} points currently selected." +
                               "\nPress the middle mouse button to restart." +
                               "\nClose the window to finish selecting points.")

            # Capture a single point from the user
            new_point, fig, axis = display_image_with_callback(temp_image_array,
                                                               image_title,
                                                               default_result,
                                                               window_name,
                                                               fig_to_plot_on=fig,
                                                               axis_to_plot_on=axis,
                                                               return_fig_and_axes=True)

            # Recopy the original image from the object passed in
            temp_image_array = copy(image_data.colorized_image)

            # If the window was closed
            if new_point == default_result:

                # Break out of the loop
                break

            # If no point was selected
            elif len(new_point) == 0:

                # Empty the lists and start over
                list_of_points = []
                list_of_line_colors = []

            # Otherwise
            else:

                # Add the newly selected point to the list of points
                list_of_points.append(new_point)

                # If more than one point has been selected
                if len(list_of_points) > 1:

                    # Generate a new line color
                    list_of_line_colors.append(intermediate_line_color)

                    # Iterate through the list of points to draw each line using the appropriate color
                    for i in range(1, len(list_of_points)):
                        temp_image_array = line(temp_image_array, list_of_points[i - 1], list_of_points[i],
                                                list_of_line_colors[i - 1], intermediate_line_width)

                    # Connect the first and last point with a different line
                    # There must be at least three points selected for this to occur
                    if len(list_of_points) > 2:
                        temp_image_array = line(temp_image_array, list_of_points[0], list_of_points[-1],
                                                final_line_color, final_line_width)

                # Draw each point selected on the image
                for point in list_of_points:
                    temp_image_array = circle(temp_image_array, point, point_radius, point_color, -1)

        # Ensure that at least three points have been selected
        if len(list_of_points) < 3:
            raise Exception("Not enough points were selected to make a polygon." +
                            " At least 3 points must be selected.")

        # Return the list of points generated by the user
        if display_result:
            print("List of points selected for " + polygon_use + ":")
            print(list_of_points)

    # Return the list of points passed in or selected
    return list_of_points


if __name__ == '__main__':

    # Create a test image
    image = ImageData(image_data=imread('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/' +
                                        'scripts/Test/Images/Series2/Slice_30.png'))

    # Recolor the image to properly replicate the system
    image.colorized_image = cvtColor(image.original_image, COLOR_BGR2GRAY)

    print(user_input_polygon_points(image, "test"))
