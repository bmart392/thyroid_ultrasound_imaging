"""
Contains the code for the stitch_image_arrays function
"""

# Import standard python packages
from numpy import append, ones, ndarray, uint8
from matplotlib.pyplot import show, imshow, imread
from cv2 import cvtColor, COLOR_BGR2GRAY, COLOR_GRAY2BGR

# Import custom python packages
from thyroid_ultrasound_imaging_support.Visualization.display_process_timer import *

# Define a thickness of the boundary lines
BOUNDARY_THICKNESS: int = 8


def stitch_image_arrays(image_arrays: list,
                        channel_selector: int = None) -> ndarray:
    """
    Creates a single array created by stitching the given arrays together with bounding lines in between and surrounding
    the entire stitched array. Individual arrays can be single or three channels. All arrays will then be converted to
    have the same number of channels as the first image.

    Parameters
    ----------
    image_arrays
        A 1D or 2D list of arrays formatted in the positions in which they should be displayed.
    channel_selector
        Select the number of channels that the returned array will have, either 1 or 3. If no value is given, use the
        number of channels in the first image.
    """

    # Find the number of rows of images to plot
    num_rows = len(image_arrays)

    # Find the number of columns of images to plot
    if type(image_arrays[0]) == list:
        num_cols = len(image_arrays[0])
    elif type(image_arrays[0]) == ndarray:
        image_arrays = [image_arrays]
        num_rows = len(image_arrays)
        num_cols = len(image_arrays[0])
    else:
        raise Exception("Improper formatting of image arrays. Type " + str(type(image_arrays)) + " is not accepted.")

    # Save the number of rows and columns in an individual array
    image_array_rows = image_arrays[0][0].shape[0]
    image_array_cols = image_arrays[0][0].shape[1]

    # Save the number of channels contained in the individual array
    if channel_selector is None:
        try:
            image_array_channels = image_arrays[0][0].shape[2]
        except IndexError:
            image_array_channels = 1
    else:
        if channel_selector == 1 or channel_selector == 3:
            image_array_channels = channel_selector
        else:
            raise Exception("Arrays cannot be returned with " + str(channel_selector) + " channels.")

    # For each row of images
    for row in range(len(image_arrays)):

        # For each column in each row
        for col in range(len(image_arrays[row])):

            # If the image array shape does not match the desired number of channels,
            # change the color scheme of the image
            if len(image_arrays[row][col].shape) == 2 and image_array_channels == 3:
                image_arrays[row][col] = cvtColor(image_arrays[row][col], COLOR_GRAY2BGR)
            elif len(image_arrays[row][col].shape) == 3 and image_array_channels == 1:
                image_arrays[row][col] = cvtColor(image_arrays[row][col], COLOR_BGR2GRAY)

    # Calculate the number of rows and columns in the stitched image
    stitched_array_num_cols = (num_cols * image_array_cols) + (BOUNDARY_THICKNESS * (num_cols - 1))
    stitched_array_num_rows = (num_rows * image_array_rows) + (BOUNDARY_THICKNESS * (num_rows + 1))

    # Calculate the shape of the boundaries around the arrays
    boundary_full_row_shape = [BOUNDARY_THICKNESS, stitched_array_num_cols]
    boundary_full_col_shape = [stitched_array_num_rows, BOUNDARY_THICKNESS]
    boundary_img_col_shape = [image_array_rows, BOUNDARY_THICKNESS]

    # Assign a third dimension to each boundary, if needed
    if image_array_channels == 3:
        boundary_full_row_shape.append(image_array_channels)
        boundary_full_col_shape.append(image_array_channels)
        boundary_img_col_shape.append(image_array_channels)

    # Create the bounding arrays
    boundary_full_row = ones(boundary_full_row_shape, dtype=uint8) * 255
    boundary_full_col = ones(boundary_full_col_shape, dtype=uint8) * 255
    boundary_img_col = ones(boundary_img_col_shape, dtype=uint8) * 255

    # Define a variable to store the result in
    result_array = None

    # For each row of arrays
    for row in range(num_rows):

        # Define an array to store the stitched row in
        row_array = None

        # For each array in the row of arrays
        for column in range(num_cols):

            # If the array for the row does not exist yet
            if row_array is None:

                # Set it as the current image
                row_array = image_arrays[row][column]

            # Otherwise
            else:

                # Append the current image to the existing row array
                row_array = append(row_array, image_arrays[row][column], 1)

            # If it is not the last image in the row,
            if not column == num_cols - 1:

                # Add a boundary to the right of the array
                row_array = append(row_array, boundary_img_col, 1)

        # If it is the first row
        if row == 0:

            # Add a boundary before the row
            row_array = append(boundary_full_row, row_array, 0)

        # Add a boundary after the row
        row_array = append(row_array, boundary_full_row, 0)

        # If the result array does not exist yet
        if result_array is None:

            # Set the current row array as the result
            result_array = row_array

        # Otherwise,
        else:

            # Add the current row to the result
            result_array = append(result_array, row_array, 0)

    # Add a boundary to the top and bottom of the resulting array
    result_array = append(boundary_full_col, result_array, 1)
    result_array = append(result_array, boundary_full_col, 1)

    return result_array


if __name__ == '__main__':

    # Define a path to the images to use for testing
    folder_path = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/Test/Images/2023-11-29_19-14'
    image_name_stub = folder_path + '/Slice_000'
    ext = '.png'

    # Create a test array of images
    test_image_arrays = [
        [imread(image_name_stub + '10' + ext), imread(image_name_stub + '20' + ext),
         imread(image_name_stub + '30' + ext)],
        [imread(image_name_stub + '40' + ext), imread(image_name_stub + '50' + ext),
         imread(image_name_stub + '60' + ext)],
    ]

    # Get the current time
    start_process_time = time()

    # Stitch them together
    test_result = stitch_image_arrays(test_image_arrays)

    # Display how long it took to convert the image
    display_process_timer(start_process_time, "Image Stitching Time")

    # Show the result
    imshow(test_result)
    show()
