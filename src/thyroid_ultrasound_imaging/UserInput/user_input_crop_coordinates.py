"""
Contains user_input_crop_coordinates and necessary callback functions.
"""
# Import standard packages
from copy import copy
from cv2 import imshow, waitKey, destroyAllWindows, \
    line, rectangle, setMouseCallback, EVENT_LBUTTONDOWN, \
    startWindowThread

# Import custom objects
from thyroid_ultrasound_imaging.ImageData.ImageData import ImageData

# Import custom functions
from thyroid_ultrasound_imaging.UserInput.display_text_on_image import display_text_on_image


def user_input_crop_coordinates(image_data: ImageData, crop_coordinates: list = None):
    """
    Prompt the user to select two coordinates to crop the image. If crop_coordinates are provided,
    do nothing and return them.

    Parameters
    ----------
    image_data
        the ImageData object containing the image to crop.
    crop_coordinates
        a list of two (x, y) coordinates.
    """

    # Do nothing if the user chooses not to crop the image.
    if crop_coordinates is None:

        # Define window name
        window_name = "Image Crop Selection"

        # Define characteristics of the shapes to show on the image
        line_color = (255, 255, 255)
        line_thickness = 2
        box_color = (255, 255, 255)
        box_thickness = 3

        # Create a copy of the original image to prevent damaging the original object
        temp_image_array = copy(image_data.original_image)

        # Add text to the image informing the user
        temp_image_array = display_text_on_image(temp_image_array,
                                                 "Select the upper left-hand crop corner." +
                                                 " Press any key to continue.")

        # Display the image for the user
        imshow(window_name, temp_image_array)

        # Define a variable to store the result of the user's click
        first_corner = [int(0), int(0)]

        # Ask user to select the first corner to use to crop the image
        setMouseCallback(window_name, click_event, first_corner)
        waitKey(0)

        # Recopy the original image to clear the previous text
        temp_image_array = copy(image_data.original_image)

        # Display two lines to show how the image has been cropped so far
        temp_image_array = line(temp_image_array, (first_corner[0], first_corner[1]),
                                (first_corner[0], temp_image_array.shape[0]),
                                line_color, line_thickness)
        temp_image_array = line(temp_image_array, (first_corner[0], first_corner[1]),
                                (temp_image_array.shape[1], first_corner[1]),
                                line_color, line_thickness)

        # Display text to assist the user
        temp_image_array = display_text_on_image(temp_image_array,
                                                 "Select the lower right-hand crop corner." +
                                                 " Press any key to continue.")
        # Show the new image to the user
        imshow(window_name, temp_image_array)

        # Define a variable to store the result of the user's click
        second_corner = [int(image_data.original_image.shape[1] - 1), int(image_data.original_image.shape[0] - 1)]

        # Ask the user to select the corner to use to crop the image
        setMouseCallback(window_name, click_event, second_corner)
        waitKey(0)

        # Ensure that the second corner was chosen properly
        if not (second_corner[0] > first_corner[0] and second_corner[1] > first_corner[1]):
            raise Exception("Second corner was not selected below and to the right of the first.")

        # Recopy the original image to clear the previous annotations
        temp_image_array = copy(image_data.original_image)

        # Draw a rectangle to show how the image will be cropped
        temp_image_array = rectangle(temp_image_array, first_corner, second_corner, box_color, box_thickness)

        # Display text to assist the user
        temp_image_array = display_text_on_image(temp_image_array,
                                                 "Preview of crop region." +
                                                 " Press any key to continue.")

        # Show the new image to the user
        imshow(window_name, temp_image_array)
        waitKey(0)

        # Destroy all windows when finished
        destroyAllWindows()
        waitKey(1)

        # Display the points selected for the user
        print("1st Corner: (" + str(first_corner[0]) + ", " + str(first_corner[1]) + ")\n" +
              "2nd Corner: (" + str(second_corner[0]) + ", " + str(second_corner[1]) + ")")

        # Return the points selected
        return [first_corner, second_corner]

    else:
        # Return the points passed in
        return crop_coordinates


def click_event(event: int, x, y, flags, additional_inputs: list):
    """
    Return the position selected on the click of a mouse.

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
        the variable declared outside this function that will be updated to include
        the new x-position and y-position.
    """

    # Do nothing until the left mouse button is clicked
    if event == EVENT_LBUTTONDOWN:
        # Save the click location
        additional_inputs[0] = int(x)
        additional_inputs[1] = int(y)
