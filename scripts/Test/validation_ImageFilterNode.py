"""
Contains the code for validating that the ImageFilterNode is successful.
"""

# Import standard python packages
from rospy import Rate
from cv2 import cvtColor, COLOR_BGR2GRAY, imshow, waitKey, COLOR_GRAY2RGB, COLOR_BGR2RGB, resize
from numpy import load, array, uint8, zeros
from matplotlib.pyplot import subplots, show, matshow, savefig

# Import standard ROS packages
from std_msgs.msg import Bool

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_image_files import \
    load_folder_of_image_files
from thyroid_ultrasound_imaging_support.ImageFilter.FilterConstants import COLOR_GRAY, COLOR_BGR, COLOR_RGB
from thyroid_ultrasound_imaging_support.Visualization.display_process_timer import display_process_timer, time
from thyroid_ultrasound_imaging_support.Visualization.stitch_image_arrays import stitch_image_arrays
from validation_constants import *
from thyroid_ultrasound_imaging_support.Visualization.create_mask_overlay_image import create_mask_overlay_array, \
    COLORIZED, PREV_IMG_MASK
from thyroid_ultrasound_imaging_support.Visualization.create_mask_display_array import create_mask_display_array

# Import custom ROS packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageData.convert_array_to_image_message import convert_array_to_image_message
from ImageFilterNode import ImageFilterNode, GRABCUT_FILTER
from thyroid_ultrasound_messages.msg import image_crop_coordinates, initialization_mask_message

# Validation Options
save_image_data = True

# Image Filter Options
blurring = [False, True]
opening = [False, True]
accuracy_of_initial_mask = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
down_sampling_rate = [1, 1 / 2, 1 / 3, 1 / 4, 1 / 5]
iteration_count = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

# Plotting axis counter
ii = 0

# Number of rows and columns for plotting
NUM_ROWS = 3
NUM_COLS = 4

# Create a figure to plot the results
fig, axes = subplots(NUM_ROWS, NUM_COLS)
fig.suptitle('Image Filter Process', fontsize=36)

# Create the filter_node to be tested
filter_node = ImageFilterNode(filter_type=GRABCUT_FILTER,
                              debug_mode=False, analysis_mode=True)

# Read in a list of images from a folder
image_start_index = 51
images_as_arrays = load_folder_of_image_files('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts'
                                              '/Test/Images/2023-11-29_19-14', image_start_index)

# Load the arrays to use for the image crop coordinates and the initialization array
crop_coords = load(INITIALIZATION_INFORMATION_PATH + CROP_FILE_NAME)
user_mask = resize(load(INITIALIZATION_INFORMATION_PATH + INITIALIZATION_FILE_NAME), (639, 403))

# Show the user mask as loaded
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
    cvtColor(user_mask * uint8(125), COLOR_GRAY2RGB))
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title(
    'Segmentation Initialization\nMask as Loaded', fontsize=24)
ii = ii + 1

# Define the maximum rate at which images should be processed
rate = Rate(5)  # Hz

# Send the image crop coordinates to use for cropping the images
image_crop_coordinates_msg = image_crop_coordinates(first_coordinate_x=crop_coords[0][0],
                                                    first_coordinate_y=crop_coords[0][1],
                                                    second_coordinate_x=crop_coords[1][0],
                                                    second_coordinate_y=crop_coords[1][1])
filter_node.crop_coordinates_callback(image_crop_coordinates_msg)

# Send the previous image mask to use for the segmentation
user_mask_msg = initialization_mask_message(previous_image_mask=convert_array_to_image_message(user_mask))
filter_node.grabcut_initialization_mask_callback(user_mask_msg)

# Display the user mask as was loaded
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
    cvtColor(filter_node.image_filter.previous_image_mask_array * uint8(125), COLOR_GRAY2RGB))
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Segmentation Initialization Mask as\n'
                                                                           'Previous Image Mask', fontsize=24)
ii = ii + 1

# Tell the image filter that it is time to filter images
filter_node.filter_images_callback(Bool(True))

filter_node.patient_contact_callback(Bool(True))

for image_array in images_as_arrays:

    # Define the previous image mask to use for the segmentation
    this_image_previous_mask = filter_node.image_filter.previous_image_mask_array

    # Create a new image data message
    new_image_data_message = ImageData(image_data=cvtColor(image_array, COLOR_BGR2GRAY),
                                       image_color=COLOR_GRAY).convert_to_message()

    # Feed the image into the filter_node
    filter_node.raw_image_callback(new_image_data_message)

    # Analyze the image
    filter_node.main_loop()

    # Display the Previous-Image Mask for this image
    imshow("Previous-Image Mask for this image",
           create_mask_overlay_array(filter_node.image_data.pre_processed_image, 3,
                                     this_image_previous_mask, COLOR_BGR,
                                     PREV_IMG_MASK, color_pr_fgd=(85, 0, 0)))
    waitKey(1)

    imshow("Current Image", filter_node.image_data.pre_processed_image)
    waitKey(1)
    imshow("Foreground", create_mask_overlay_array(filter_node.image_data.pre_processed_image, 3,
                                                   filter_node.image_data.image_mask, COLOR_BGR,
                                                   COLORIZED, overlay_color=(34, 25, 0)))
    waitKey(1)
    imshow("Previous-Image Mask for next image",
           create_mask_overlay_array(filter_node.image_data.pre_processed_image, 3,
                                     filter_node.image_filter.previous_image_mask_array, COLOR_BGR,
                                     PREV_IMG_MASK, color_pr_fgd=(85, 0, 0)))
    waitKey(1)

    rate.sleep()


# If this script is testing the save data functionality
if save_image_data:
    # Note the time the process started
    start_time = time()

    # Convert the image data to a zipped folder
    object_location = filter_node.image_data.save_object_to_file(IMAGE_DATA_OBJECTS_PATH)

    # Calculate how long it took to save the image
    start_time = display_process_timer(start_time, "Object Saving Time")

    # Rebuild the data from the file
    result_object = ImageData(image_data_location=object_location)

    # Calculate how long it took to rebuild the data
    display_process_timer(start_time, "Object Recreation Time")
else:
    # Otherwise just look at the data in the node
    result_object = filter_node.image_data

# Show the original image
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
    cvtColor(result_object.original_image, COLOR_GRAY2RGB))
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Original Image', fontsize=24)
ii = ii + 1

axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
    cvtColor(result_object.cropped_image, COLOR_GRAY2RGB))
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Cropped Image', fontsize=24)
ii = ii + 1

axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
    cvtColor(result_object.colorized_image, COLOR_BGR2RGB))
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Colorized Image', fontsize=24)
ii = ii + 1

axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
    cvtColor(result_object.down_sampled_image, COLOR_BGR2RGB))
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Down-sampled Image', fontsize=24)
ii = ii + 1

axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
    cvtColor(result_object.pre_processed_image, COLOR_BGR2RGB))
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Pre-processed Image', fontsize=24)
ii = ii + 1

axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
    cvtColor(result_object.image_mask * uint8(125), COLOR_GRAY2RGB))
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Result Image Mask', fontsize=24)
ii = ii + 1

axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
    cvtColor(result_object.post_processed_mask * uint8(125), COLOR_GRAY2RGB))
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Post-processed Image Mask', fontsize=24)
ii = ii + 1

axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
    cvtColor(filter_node.image_filter.previous_image_mask_array * uint8(125), COLOR_GRAY2RGB))
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Previous Image Mask', fontsize=24)
ii = ii + 1

#
# imshow('Pre-processed Image', filter_node.image_data.pre_processed_image)
# waitKey(1)
#
# imshow('Previous Image Mask', filter_node.image_data.segmentation_initialization_mask)
# waitKey(1)
#
# imshow('Image Mask', filter_node.image_data.image_mask)
fig.set_size_inches(35, 20)
savefig(RESULTS_PATH + 'result.png', bbox_inches='tight', dpi=300)
show()
print('Done')

# # Define image series names
# SERIES_1: int = 1
# SERIES_2: int = 2
#
#
# # Define several controls to ease testing
# capture_user_input = True  # True = define input, false = use values
# image_series_to_use = SERIES_2  # See above for options
# imaging_mode = IMG_CONTINUOUS  # Single or continuous
#
# # Define the visualizations to display for each filter
# list_of_visualizations = [
#     [
#         # SHOW_ORIGINAL,
#         SHOW_CROPPED,
#         # SHOW_RECOLOR,
#         # SHOW_BLUR,
#         SHOW_MASK,
#         SHOW_EXPANDED_MASK,
#         # SHOW_FOREGROUND,
#         # SHOW_SURE_FOREGROUND,
#         # SHOW_SURE_BACKGROUND,
#         # SHOW_PROBABLE_FOREGROUND,
#         # SHOW_INITIALIZED_MASK,
#         SHOW_CENTROIDS_ONLY,
#         # SHOW_CENTROIDS_CROSS_ONLY,
#
#         # TODO Implement following mode
#         # SHOW_MASK_OVERLAY,
#
#         # TODO Implement following mode,
#         # SHOW_CENTROIDS_CROSS_OVERLAY,
#
#         # TODO Implement following mode
#         # SHOW_MASK_CENTROIDS_CROSS_OVERLAY
#     ],
#     [
#         # SHOW_ORIGINAL,
#         # SHOW_CROPPED,
#         # SHOW_BLUR,
#         # SHOW_MASK,
#         # SHOW_INITIALIZED_MASK,
#         # SHOW_GRABCUT_USER_INITIALIZATION_0,
#         # SHOW_EXPANDED_MASK,
#         # SHOW_FOREGROUND,
#         # SHOW_SURE_FOREGROUND,
#         # SHOW_SURE_BACKGROUND,
#         # SHOW_PROBABLE_FOREGROUND
#         # SHOW_CENTROIDS_CROSS_ONLY,
#         # SHOW_MASK_OVERLAY
#     ]
# ]
#
# # Create the visualization objects
# visualizations = [Visualization(imaging_mode, list_of_visualizations[0],
#                                 visualization_title="Grabcut Filter", num_cols_in_fig=3),
#                   Visualization(imaging_mode, list_of_visualizations[1],
#                                 visualization_title="Threshold Filter", num_cols_in_fig=3),
#                   ]
#
# # Read in images from the image series as arrays of image data objects
# # Define saved image parameters for each series
# if image_series_to_use == SERIES_1:
#
#     images = import_images_as_image_data_objects('Images/Series1', COLOR_BGR, 30)  # Read images from Series 1
#
#     image_crop_coordinates = [[328, 67], [540, 578]]  # Series 1 - Slice 30
#     list_of_points_for_background_polygon = [(21, 52), (183, 58), (173, 252), (27, 236)]  # Series 1 - Slice 30
#     list_of_points_for_foreground_polygon = [(102, 292), (157, 318), (101, 345)]  # Series 1 - Slice 30
#
#     thresholding_parameters = (100, 120)  # Series 1 - Slice 30
#
# elif image_series_to_use == SERIES_2:
#
#     images = import_images_as_image_data_objects('Images/Series2', COLOR_BGR, 20)  # Read images from Series 2
#
#     image_crop_coordinates = [[198, 197], [538, 494]]  # Series 2 - Slice 20
#     list_of_points_for_background_polygon = [(2, 2), (334, 6), (321, 131), (163, 134), (49, 142),
#                                              (23, 179)]  # Series 2 - Slice 20
#     list_of_points_for_foreground_polygon = [(27, 212), (128, 157), (58, 259)]  # Series 2 - Slice 20
#
#     thresholding_parameters = (69, 124)  # Series 2 - Slice 20
#
# else:
#     raise Exception("Series choice not recognized.")
#
# # Save an image for the user to annotate and to create the initial mask
# test_image = images[0]
#
# # If user input is required, overwrite the saved parameters for the series
# if capture_user_input:
#
#     # Capture the image crop coordinates from the user
#     image_crop_coordinates = user_input_crop_coordinates(test_image)
#
#     # Create an ImageFilter object to crop the image so that the initial mask can be generated
#     image_filter = ImageFilter(image_crop_coordinates=image_crop_coordinates)
#
#     # Crop the test image
#     image_filter.crop_image(test_image)
#
#     image_filter.colorize_image(test_image)
#
#     # Capture the background of the image from the user
#     list_of_points_for_background_polygon = user_input_polygon_points(test_image, "background",
#                                                                       display_result=True)
#
#     # Capture the foreground of the image from the user
#     list_of_points_for_foreground_polygon = user_input_polygon_points(test_image, "foreground",
#                                                                       display_result=True)
#
#     # Use the user input to generate the thresholding values for the threshold filter
#     thresholding_parameters = get_threshold_values_from_user_input(test_image,
#                                                                    num_standard_deviations=1.75,
#                                                                    display_result=True)
#
# # Create an ImageFilter object to crop the image so that the initial mask can be generated
# image_filter = ImageFilter(image_crop_coordinates=image_crop_coordinates)
#
# # Crop the test image
# image_filter.crop_image(test_image)
#
# # Convert the points of the background and foreground polygons to triangles
# list_of_background_triangles = create_convex_triangles_from_points(list_of_points_for_background_polygon)
# list_of_foreground_triangles = create_convex_triangles_from_points(list_of_points_for_foreground_polygon)
#
# # Use the polygons to generate the initial mask
# initial_mask = create_mask_array_from_triangles(list_of_background_triangles, list_of_foreground_triangles,
#                                                 test_image.cropped_image.shape[:2])
#
# # Create the image filters
# image_filters = [ImageFilterGrabCut(previous_mask_array=initial_mask,
#                                     analysis_mode=False,
#                                     image_crop_included=True,
#                                     image_crop_coordinates=image_crop_coordinates,
#                                     increase_contrast=False
#                                     ),
#                  ImageFilterThreshold(analysis_mode=False,
#                                       image_crop_included=True,
#                                       increase_contrast=False,
#                                       image_crop_coordinates=image_crop_coordinates,
#                                       thresholding_parameters=thresholding_parameters),
#                  ]
#
# # Initialize an empty array to hold the confidence map
# confidence_map = np.zeros(test_image.original_image.shape[:2])
#
# # Filter each image in the series
# for image_data in images:
#
#     # Reduce the confidence of all cells in the map due to time having elapsed
#     # TODO the temporal confidence value will need tuning
#     confidence_map = confidence_map * .25  # This is the temporal confidence
#
#     # For each image filter and visualization pair, segment the image and show the results
#     for image_filter, visualization in zip(image_filters, visualizations):
#
#         # Create a temporary copy of the image data
#         temp_image_data: ImageData = copy(image_data)
#
#         # Fully filter the image data
#         image_filter.basic_filter_image(temp_image_data)
#
#         # Find the contours in the image
#         temp_image_data.generate_contours_in_image()
#
#         # Find the centroids of the contours in the image
#         temp_image_data.calculate_image_centroids()
#
#         # Visualize the result
#         visualization.visualize_images(temp_image_data)
#
#         # Update the confidence map with the new data acquired by the segmentations
#         # TODO the confidence weights for each filter will need tuning
#         if type(image_filter) is ImageFilterGrabCut:
#             confidence_map = confidence_map + 0.375 * temp_image_data.expanded_image_mask
#         if type(image_filter) is ImageFilterThreshold:
#             confidence_map = confidence_map + 0.375 * temp_image_data.expanded_image_mask
#
#     # Show the updated confidence map
#     # cv2.imshow("Confidence Map", np.uint8(confidence_map * 255))
#     # cv2.waitKey(1)
#
#     # Overlay the confidence map on to the original image
#     # TODO the confidence map should be overlaid onto the original image
#     """cv2.imshow("Confidence Map Segmentation",
#                np.uint8(visualization.create_mask_overlay_array(temp_image_data.original_image,
#                                                        confidence_map, 1)))"""
