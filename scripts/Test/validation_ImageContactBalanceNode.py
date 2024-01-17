"""
Contains the code for validating that the ImageContactBalanceNode is successful.
"""

# Import standard python packages
from cv2 import cvtColor, COLOR_BGR2GRAY, circle, line, imshow, waitKey, VideoWriter, VideoWriter_fourcc
from numpy import zeros, uint8
from rospy import Rate

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_image_files import \
    load_folder_of_image_files
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData

# Import custom ROS packages
from ImageContactBalanceNode import ImageContactBalanceNode, LEFT_SECTOR, RIGHT_SECTOR, X_POINTS, Y_POINTS, \
    CURRENT_VALUE, NEW_VALUE


# Create the test node
test_node = ImageContactBalanceNode()

# Read in a list of images from a folder
image_start_index = 1
images_as_arrays = load_folder_of_image_files('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts'
                                                 '/Test/Images/2023-11-29_19-14', image_start_index)

# Define the rate at which images are shown
rate = Rate(10)  # Hz

# Set the image shape so that the image sectors can be built
test_node.img_num_rows[NEW_VALUE] = images_as_arrays[0].shape[0]
test_node.img_num_cols[NEW_VALUE] = images_as_arrays[0].shape[1]

# Rebuild the sectors based on the images saved
test_node.rebuild_sectors()

# Define an array to build a mask of the sectors in
sector_mask = zeros(images_as_arrays[0].shape, dtype=uint8)

# Define the colors to use for each sector
sector_colors = []
color_options = [(35, 0, 0),
                 (0, 25, 0),
                 (0, 0, 0),
                 (0, 0, 25)]

# Build a list of colors to use for the sectors
for ii in range(len(test_node.sectors)):
    sector_colors.append(color_options[ii % 4])

# Define a variable to use to store the last sector where a point was found
last_sector_used = 0

# Iterate through the rows of the original image that contain the ultrasound image
for yy in range(test_node.start_of_us_image[CURRENT_VALUE],
                test_node.img_num_rows[CURRENT_VALUE] - test_node.end_of_us_image_from_bottom[CURRENT_VALUE]):

    # Iterate through the columns that are between the left line of the leftmost column and the right
    # line of the rightmost column skipping points based on the down-sampling rate
    for xx in range(round(test_node.sectors[0].left_line.calc_point_on_line(y_value=yy)) - 2,
                    round(test_node.sectors[-1].right_line.calc_point_on_line(y_value=yy)) + 2):

        # Window the acceptable values of x between 0 and the number of columns in the image
        if xx < 0:
            xx = 0
        if xx > test_node.img_num_cols[CURRENT_VALUE] - 1:
            xx = test_node.img_num_cols[CURRENT_VALUE] - 1

        # For each sector, starting at the sector in which the last point was found
        for jj in range(last_sector_used, len(test_node.sectors)):

            # Check if the point is in the sector
            if test_node.sectors[jj].is_point_in_sector((xx, yy), 0):

                # Update the sector mask with the corresponding color
                sector_mask[yy, xx] = sector_mask[yy, xx] + sector_colors[jj]

                # Update the value of the last sector used
                last_sector_used = jj

                break

    # Reset the value of the last sector used
    last_sector_used = 0

# Create an object to use to write the results to a video
output = VideoWriter('/home/ben/Pictures/ImageBalanceValidation.avi', VideoWriter_fourcc(*'MJPG'),
                     20, (int(images_as_arrays[0].shape[1]), int(images_as_arrays[0].shape[0])))

# For each image
for image_array in images_as_arrays:

    # Create a new image data message
    new_image_data_message = ImageData(image_data=cvtColor(image_array, COLOR_BGR2GRAY)).convert_to_message()

    # Feed it into the node through the callback function
    test_node.new_raw_image_callback(new_image_data_message)

    # Analyze the image data message
    test_node.main_loop()

    # Define an image that can be modified for visualization
    visualization_image = image_array + sector_mask

    # Draw the line of best fit calculated by the node
    visualization_image = line(visualization_image,
                               (test_node.skin_points[LEFT_SECTOR][X_POINTS][0],
                                round((test_node.skin_points[LEFT_SECTOR][X_POINTS][0] *
                                       test_node.best_fit_shared_line_a)
                                + test_node.best_fit_shared_line_b)),
                               (test_node.skin_points[RIGHT_SECTOR][X_POINTS][-1],
                                round((test_node.skin_points[RIGHT_SECTOR][X_POINTS][-1] *
                                       test_node.best_fit_shared_line_a)
                                + test_node.best_fit_shared_line_b)),
                               (0, 255, 0),
                               2)

    # Plot each skin contact point using a different color for the left and right-hand sides of the image
    for half_sector, point_color in zip(test_node.skin_points, [(0, 165, 255), (0, 255, 255)]):
        for xx, yy in zip(half_sector[X_POINTS], half_sector[Y_POINTS]):
            visualization_image = circle(visualization_image, (xx, yy), 4, point_color, thickness=-1)

    # Write the current image to a video
    output.write(visualization_image)

    # Display the images for verification
    imshow("Ultrasound Image Sectors, Skin Points, and Skin Estimation Line", visualization_image)
    waitKey(1)

    # Wait to show the next image
    rate.sleep()

output.release()
