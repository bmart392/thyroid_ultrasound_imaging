"""
Contains user_input_crop_coordinates and necessary callback functions.
"""
# Import standard packages
from copy import copy
from cv2 import line, rectangle

# Import custom objects
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData

# Import custom functions
from thyroid_ultrasound_imaging_support.UserInput.display_image_with_callback import display_image_with_callback


def user_input_crop_coordinates(image_data: ImageData,
                                crop_coordinates: list = None):
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

        while True:

            # Define window name
            window_name = "Image Crop Selection"

            # Define characteristics of the shapes to show on the image
            line_color = (255, 255, 255)
            line_thickness = 2
            box_color = (255, 255, 255)
            box_thickness = 3

            # Create a copy of the original image to prevent damaging the original object
            temp_image_array = copy(image_data.original_image)

            # Define a variable to store the result of the user's click
            first_corner = [int(0), int(0)]

            # Display the image and get the users input
            first_corner = display_image_with_callback(temp_image_array,
                                                       "Select the upper left-hand crop corner." +
                                                       "\n Press the middle mouse button to restart.",
                                                       first_corner,
                                                       window_name,
                                                       )

            # Go back to the top of the loop if no point was selected
            if len(first_corner) == 0:
                continue

            # Recopy the original image to clear the previous text
            temp_image_array = copy(image_data.original_image)

            # Display two lines to show how the image has been cropped so far
            temp_image_array = line(temp_image_array, (first_corner[0], first_corner[1]),
                                    (first_corner[0], temp_image_array.shape[0]),
                                    line_color, line_thickness)
            temp_image_array = line(temp_image_array, (first_corner[0], first_corner[1]),
                                    (temp_image_array.shape[1], first_corner[1]),
                                    line_color, line_thickness)

            # Define a variable to store the result of the user's click
            second_corner = [int(image_data.original_image.shape[1] - 1), int(image_data.original_image.shape[0] - 1)]

            # Display the image and get the users input
            second_corner = display_image_with_callback(temp_image_array,
                                                        "Select the lower right-hand crop corner." +
                                                        "\nPress the middle mouse button to restart.",
                                                        second_corner,
                                                        window_name,
                                                        )

            # Go back to the top of the loop if no point was selected
            if len(second_corner) == 0:
                continue

            # Ensure that the second corner was chosen properly
            if not (second_corner[0] > first_corner[0] and second_corner[1] > first_corner[1]):
                print("Second corner was not selected below and to the right of the first.")
            else:

                # Recopy the original image to clear the previous annotations
                temp_image_array = copy(image_data.original_image)

                # Draw a rectangle to show how the image will be cropped
                temp_image_array = rectangle(temp_image_array, first_corner, second_corner, box_color, box_thickness)

                # Display the image and get the user's approval
                approval = display_image_with_callback(temp_image_array,
                                                       "Preview of crop region." +
                                                       "\nClick on the image to approve." +
                                                       "\nPress the middle mouse button to restart.",
                                                       [],
                                                       window_name)
                # If a point was selected
                if len(approval) == 2:

                    # Display the points selected for the user
                    print("1st Corner: (" + str(first_corner[0]) + ", " + str(first_corner[1]) + ")\n" +
                          "2nd Corner: (" + str(second_corner[0]) + ", " + str(second_corner[1]) + ")")

                    # Break out of the function
                    break

                # Otherwise loop around to try again

        # Return the points selected
        return [first_corner, second_corner]

    else:
        # Return the points passed in
        return crop_coordinates


if __name__ == '__main__':

    # Create a test image
    image = ImageData(image_filepath='/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/' +
                                     'scripts/Test/Images/Series2/Slice_30.png')

    # Test the function
    user_input_crop_coordinates(image)
