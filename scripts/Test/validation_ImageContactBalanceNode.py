"""
Contains the code for validating that the ImageContactBalanceNode is successful.
"""

# Import standard python packages
from cv2 import cvtColor, COLOR_BGR2GRAY, circle, line, imshow, waitKey, VideoWriter, VideoWriter_fourcc, imread, imwrite
from numpy import zeros, uint8
from rospy import Rate
from copy import copy

# Import standard ROS packages
from std_msgs.msg import Float64

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_image_files import \
    load_folder_of_image_files
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData

# Import custom ROS packages
from ImageContactBalanceNode import ImageContactBalanceNode, LEFT_SECTOR, RIGHT_SECTOR, X_POINTS, Y_POINTS, \
    CURRENT_VALUE, NEW_VALUE
from thyroid_ultrasound_imaging_support.RegisteredData.generate_ordered_list_of_directory_contents import \
    generate_ordered_list_of_directory_contents

# Define the base path for image saving
IMAGE_SAVING_BASE_PATH: str = '/home/ben/Documents/Thesis/Visuals/'
# Create the test node
test_node = ImageContactBalanceNode()
test_node.image_depth_callback(Float64(6.0))

# Read in a list of images from a folder
image_start_index = 1
images_as_arrays = [imread(path) for path in generate_ordered_list_of_directory_contents(
    directory_path='/home/ben/thyroid_ultrasound_data/experimentation/2024-03-14_13-54-28-386172_experiment/'
                   'RawImages_2024-03-14_13-54-28-386172',
    sort_indices=(0, 1))]

# Define the rate at which images are shown
rate = Rate(60)  # Hz

# Set the image shape so that the image sectors can be built
test_node.img_num_rows[NEW_VALUE] = images_as_arrays[0].shape[0]
test_node.img_num_cols[NEW_VALUE] = images_as_arrays[0].shape[1]

# Rebuild the sectors based on the images saved
test_node.rebuild_sectors()

# Define an array to build a mask of the sectors in
sector_mask = zeros(images_as_arrays[0].shape, dtype=uint8)

# Define the colors to use for each sector
sector_colors = []
color_options = [(25, 25, 25), (0, 0, 0)]  # BGR colors

# Build a list of colors to use for the sectors
for ii in range(len(test_node.sectors)):
    sector_colors.append(color_options[ii % len(color_options)])

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
# output = VideoWriter('/home/ben/Pictures/ImageBalanceValidation.avi', VideoWriter_fourcc(*'MJPG'),
#                      20, (int(images_as_arrays[0].shape[1]), int(images_as_arrays[0].shape[0])))


# Save just the ultrasound window
ultrasound_window_mask = zeros(images_as_arrays[0].shape, dtype=uint8)

for yy in range(sector_mask.shape[0]):
    for xx in range(sector_mask.shape[1]):
        if sum(sector_mask[yy, xx]) == 0:
            ultrasound_window_mask[yy, xx] = (255, 255, 255)

# Save the original image for convenience
# imwrite(IMAGE_SAVING_BASE_PATH + 'ImageBalance_Original.png', images_as_arrays[0])

# Save just the ultrasound window
# imwrite(IMAGE_SAVING_BASE_PATH + 'ImageBalance_UltrasoundWindow.png', images_as_arrays[0] + ultrasound_window_mask)

# Save the sector mask
# imwrite(IMAGE_SAVING_BASE_PATH + 'ImageBalance_Sectors.png', images_as_arrays[0] + sector_mask + ultrasound_window_mask)

# Define an indexer
image_indexer = 0

# For each image
for image_array in images_as_arrays:

    # Create a new image data message
    new_image_data_message = ImageData(image_data=cvtColor(image_array, COLOR_BGR2GRAY)).convert_to_message()

    # Feed it into the node through the callback function
    test_node.new_raw_image_callback(new_image_data_message)

    # Analyze the image data message
    test_node.main_loop()

    # Define an image that can be modified for visualization
    visualization_image = image_array + sector_mask + ultrasound_window_mask

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

    # # Write the current image to a video
    # output.write(visualization_image)

    if image_indexer == 0 or image_indexer == len(images_as_arrays) - 1:
        # Save the result from the
        # imwrite(IMAGE_SAVING_BASE_PATH + 'ImageBalance_SkinPoints_' + str(image_indexer) + '.png', visualization_image)
        pass

    # Display the images for verification
    imshow("Ultrasound Image Sectors, Skin Points, and Skin Estimation Line", visualization_image)
    waitKey(1)

    # Increment the image indexer
    image_indexer = image_indexer + 1

    # Wait to show the next image
    rate.sleep()

# output.release()
