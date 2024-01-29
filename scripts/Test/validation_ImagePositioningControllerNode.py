# Import standard python packages
from matplotlib.pyplot import subplots, show

# Import standard ROS packages
from std_msgs.msg import Float64

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_saved_image_data import \
    load_folder_of_saved_image_data
from thyroid_ultrasound_imaging_support.Visualization.create_mask_display_array import create_mask_display_array, \
    COLOR_RGB
from thyroid_ultrasound_imaging_support.Visualization.create_mask_overlay_image import create_mask_overlay_array
from thyroid_ultrasound_imaging_support.Visualization.add_cross_and_center_lines_on_image import \
    add_cross_and_center_lines_on_image

# Import custom ROS packages
from ImagePositioningControllerNode import ImagePositioningControllerNode

########################
# Prepare the validation
# region
# Define the location where the image data folders are located
IMAGE_DATA_LOCATION: str = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/Test/ImageData'

# Load multiple image data objects
image_data_objects = load_folder_of_saved_image_data(IMAGE_DATA_LOCATION)

# Number of rows and columns for plotting
NUM_ROWS = 2
NUM_COLS = 2

# Create the node to test
test_node = ImagePositioningControllerNode()

# endregion
########################

####################################
# Test the image depth functionality
# region

# Send a new depth to the node
test_node.imaging_depth_callback(Float64(10.))

# Reset the value back to the correct number
test_node.imaging_depth_callback(Float64(5.))

# endregion
####################################

# For each image data object
for image_data_object in image_data_objects:
    # Plotting axis counter
    ii = 0

    # Create a figure to plot the results
    fig, axes = subplots(NUM_ROWS, NUM_COLS, figsize=(24, 16))

    # Convert it to a message
    image_data_object: ImageData
    image_data_object_as_msg = image_data_object.convert_to_message()

    # Send the message to the node
    test_node.filtered_image_callback(image_data_object_as_msg)

    #   Call the main loop
    this_position_error, is_image_centered = test_node.publish_position_error()

    # Get the resolution from the controller
    image_resolution = test_node.image_positioning_controller.calculate_resolution(
        image_data_object.image_size_y) * image_data_object.cropped_image.shape[1] / \
                       image_data_object.down_sampled_image.shape[1]

    # Back calculate the pixel error
    this_pixel_error = round(this_position_error[0] / image_resolution, 3)

    # Display the image error in pixels, the distance per pixel, and the distance error
    fig.suptitle('Image Positioning Process' + '\n' +
                 'Pixel error = ' + str(this_pixel_error) + ' | ' +
                 'Distance per pixel (mm) = ' + str(round(image_resolution * 1000, 3)) + ' | ' +
                 'Position error (mm) = ' + str(round(this_position_error[0] * 1000, 3)) + ' | ' +
                 'Image Centered? ' + str(is_image_centered), fontsize=24)

    # Display the original image
    axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(image_data_object.colorized_image)
    axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Original Image', fontsize=24)
    ii = ii + 1

    # Display the pre-processed image
    axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(image_data_object.pre_processed_image)
    axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Pre-processed Image', fontsize=24)
    ii = ii + 1

    # Display the result image mask
    axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
        create_mask_display_array(image_data_object.image_mask, output_color_space=COLOR_RGB))
    axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Pre-processed Image', fontsize=24)
    ii = ii + 1

    #   Display the foreground overlaid on the pre-processed image overlaid with the centroid and lines
    axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
        add_cross_and_center_lines_on_image(
            create_mask_overlay_array(image_data_object.pre_processed_image, image_data_object.image_mask),
            image_data_object)
    )
    axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Pre-processed Image', fontsize=24)
    ii = ii + 1

    # Show the figure
    show()
