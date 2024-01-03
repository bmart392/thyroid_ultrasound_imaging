"""
Contains the code necessary for experimentally determining the effectiveness of the image filtering algorithm.
"""
from _csv import writer
from copy import copy

from cv2 import cvtColor, COLOR_BGR2GRAY, resize, GC_PR_BGD, GC_BGD, GC_FGD, COLOR_GRAY2RGB
from matplotlib.pyplot import matshow, subplots
from numpy import load, where, uint8, array

from std_msgs.msg import Bool
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageData.convert_array_to_image_message import convert_array_to_image_message
from thyroid_ultrasound_imaging_support.ImageFilter.FilterConstants import COLOR_GRAY
from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilter import UP_SAMPLING_MODE
from thyroid_ultrasound_imaging_support.ImageManipulation.generate_list_of_image_arrays import \
    generate_list_of_image_arrays
from thyroid_ultrasound_imaging_support.Validation.build_previous_image_mask_from_ground_truth import \
    build_previous_image_mask_from_ground_truth
from thyroid_ultrasound_imaging_support.Validation.calculate_dice_score import calculate_dice_score
from thyroid_ultrasound_imaging_support.Validation.create_dice_score_mask import create_dice_score_mask
from ImageFilterNode import ImageFilterNode, GRABCUT_FILTER, SHOW_ORIGINAL, SHOW_FOREGROUND, SHOW_CENTROIDS_ONLY
from thyroid_ultrasound_imaging_support.Visualization.display_process_timer import display_process_timer, time
from thyroid_ultrasound_messages.msg import image_crop_coordinates, initialization_mask_message

# Imports

# Data sources
IMAGE_SOURCE = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
               '/Test/Images/Experiment_2024-01-02'
CROP_SOURCE = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
              '/Test/Experimentation/crop_coordinates.npy'
PREVIOUS_IMAGE_MASK_SOURCE = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
                             '/Test/Experimentation/initialization_array.npy'
GROUND_TRUTH_SOURCE = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
                      '/Test/Experimentation/ground_truth.npy'
RESULTS_DESTINATION = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
                      '/Test/Experimentation/results.csv'

# Plotting axis counter
ii = 0

# Number of rows and columns for plotting
NUM_ROWS = 3
NUM_COLS = 4

# Create a figure to plot the results
fig, axes = subplots(NUM_ROWS, NUM_COLS)
fig.suptitle('Image Filter Process', fontsize=36)
# fig.show()

# Create the filter_node to be tested
filter_node = ImageFilterNode(filter_type=GRABCUT_FILTER,
                              visualizations_included=[SHOW_ORIGINAL, SHOW_FOREGROUND, SHOW_CENTROIDS_ONLY],
                              debug_mode=False, analysis_mode=False)

# Tell the image filter that it is time to filter images
filter_node.filter_images_callback(Bool(True))
filter_node.patient_contact_callback(Bool(True))

# Define image filter parameters that will be tested
blurring = [False, True]
opening = [False, True]
accuracies_of_initial_mask = [0.3, 0.6, 0.9]
down_sampling_rates = [1, 1/2, 1/3, 1/4, 1/5]
iteration_counts = [1, 5, 10, 15]

num_tests = len(accuracies_of_initial_mask) * len(down_sampling_rates) * len(iteration_counts)

# Load the image that will be segmented
image_start_index = 50
images_as_arrays = generate_list_of_image_arrays(IMAGE_SOURCE, image_start_index)
image_for_test = images_as_arrays[0]

# Show the previous image mask as loaded
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(image_for_test)
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Original Image\n as Loaded', fontsize=24)
ii = ii + 1

# Load the crop coordinates
crop_coordinates = load(CROP_SOURCE)

# Send the image crop coordinates to use for cropping the images
image_crop_coordinates_msg = image_crop_coordinates(first_coordinate_x=crop_coordinates[0][0],
                                                    first_coordinate_y=crop_coordinates[0][1],
                                                    second_coordinate_x=crop_coordinates[1][0],
                                                    second_coordinate_y=crop_coordinates[1][1])
filter_node.crop_coordinates_callback(image_crop_coordinates_msg)

# Create a new image data message
new_image_data = ImageData(image_data=cvtColor(copy(image_for_test), COLOR_BGR2GRAY),
                           image_color=COLOR_GRAY)
filter_node.image_filter.crop_image(new_image_data)

# Load the previous image mask that contains the ground truth segmentation
ground_truth_mask = load(GROUND_TRUTH_SOURCE)

# Show the ground truth mask as it was loaded
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
    cvtColor(ground_truth_mask, COLOR_GRAY2RGB) * array([25, 0, 0]).astype('uint8') +
    cvtColor(new_image_data.cropped_image, COLOR_GRAY2RGB))
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title(
    'Ground Truth\nMask as Loaded', fontsize=24)
ii = ii + 1

# Define a variable to store each specific mask
dice_images = {}

# For each initialization mask score,
# TODO - HIGH - Add a way to make this a variable in my testing
for accuracy_of_initial_mask in accuracies_of_initial_mask:
    # Generate an initialization mask with a matching score and save it in the dictionary
    dice_images[accuracy_of_initial_mask] = build_previous_image_mask_from_ground_truth(ground_truth_mask,
                                                                                        accuracy_of_initial_mask,
                                                                                        1.0)

# Show the ground truth mask as it was loaded
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
    cvtColor(dice_images[accuracies_of_initial_mask[0]], COLOR_GRAY2RGB) * array([25, 0, 0]).astype('uint8') +
    cvtColor(new_image_data.cropped_image, COLOR_GRAY2RGB))
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title(
    'Previous Image Mask\nas Generated', fontsize=24)
ii = ii + 1

# Generate a csv file that will be used to save the data from the test
results_file = open(RESULTS_DESTINATION, mode='w')
results_file_writer = writer(results_file, delimiter=',')

# Define a counter to count which test is being conducted
test_number = 0

# For each possible combination
for accuracy_of_initial_mask in accuracies_of_initial_mask:
    for down_sampling_rate in down_sampling_rates:
        for iteration_count in iteration_counts:

            # TODO - HIGH - Test each set of parameters multiple times

            # Add the test parameters to the data to save
            row_data = [test_number, accuracy_of_initial_mask, down_sampling_rate, iteration_count]

            # Populate the relevant filter parameters in the image filter
            filter_node.image_filter.down_sampling_rate = down_sampling_rate
            filter_node.image_filter.segmentation_iteration_count = iteration_count
            previous_image_mask_msg = initialization_mask_message(previous_image_mask=convert_array_to_image_message(
                dice_images[accuracy_of_initial_mask]))
            filter_node.grabcut_initialization_mask_callback(previous_image_mask_msg)

            # Create a new image data message
            new_image_data_message = ImageData(image_data=cvtColor(copy(image_for_test), COLOR_BGR2GRAY),
                                               image_color=COLOR_GRAY).convert_to_message()

            # Feed the image into the filter_node
            filter_node.raw_image_callback(new_image_data_message)

            # Mark the start time
            start_time = time()

            # Analyze the image
            filter_node.main_loop()

            # Save how long it took to filter the image
            elapsed_time = display_process_timer(start_time, "", mode=False, return_time=True)

            # Calculate the DICE score of the result of the segmentation
            result_dice_score = calculate_dice_score(ground_truth_mask,
                                                     resize(filter_node.image_data.image_mask,
                                                            dsize=(ground_truth_mask.shape[1],
                                                                   ground_truth_mask.shape[0]),
                                                            # fx=1 / down_sampling_rate, fy=1 / down_sampling_rate,
                                                            interpolation=UP_SAMPLING_MODE))

            # Save the accuracy statistics
            row_data = row_data + [elapsed_time[1], result_dice_score]
            results_file_writer.writerow(row_data)

            test_number = test_number + 1

            print('Test ' + str(test_number) + ' of ' + str(num_tests) + ' completed')

# Close the results file
results_file.close()
