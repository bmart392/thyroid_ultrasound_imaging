"""
Contains the code necessary for experimentally determining the effectiveness of the image filtering algorithm.
"""

# Import standard python packages
from _csv import writer
from copy import copy
from cv2 import cvtColor, COLOR_BGR2GRAY, resize, GC_PR_BGD, GC_BGD, GC_FGD, COLOR_GRAY2RGB, imshow, waitKey, imwrite
from matplotlib.pyplot import matshow, subplots
from numpy import load, where, uint8, array, median, average, std, zeros
from os.path import isdir
from os import mkdir

# Import standard ROS packages
from std_msgs.msg import Bool

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageData.convert_array_to_image_message import convert_array_to_image_message
from thyroid_ultrasound_imaging_support.ImageFilter.FilterConstants import COLOR_GRAY, COLOR_BGR, COLOR_RGB
from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilter import UP_SAMPLING_MODE
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_image_files import \
    load_folder_of_image_files
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_numpy_arrays import \
    load_folder_of_numpy_arrays
from thyroid_ultrasound_imaging_support.Validation.build_previous_image_mask_from_ground_truth import \
    build_previous_image_mask_from_ground_truth
from thyroid_ultrasound_imaging_support.Validation.calculate_dice_score import calculate_dice_score
from thyroid_ultrasound_imaging_support.Validation.create_dice_score_mask import create_dice_score_mask
from thyroid_ultrasound_imaging_support.Visualization.create_mask_display_array import create_mask_display_array
from thyroid_ultrasound_imaging_support.Visualization.display_process_timer import display_process_timer, time
from thyroid_ultrasound_imaging_support.Validation.date_stamp_str import date_stamp_str
from thyroid_ultrasound_imaging_support.Visualization.create_mask_overlay_image import create_mask_overlay_array, \
    FADED, COLORIZED, PREV_IMG_MASK
from experimentation_column_headers import TEST_NUM, IMAGE_NUM, INITIAL_MASK_DICE_SCORE, DOWN_SAMPLING_RATE, \
    NUM_ITERATIONS, PROB_BKGD_EXP_FACTOR, SEGMENTATION_TIME_MEDIAN, SEGMENTATION_TIME_AVERAGE, \
    SEGMENTATION_TIME_STD_DEV, DICE_SCORE_MEDIAN, DICE_SCORE_AVERAGE, DICE_SCORE_STD_DEV

# import custom ROS packages
from ImageFilterNode import ImageFilterNode, GRABCUT_FILTER
from thyroid_ultrasound_messages.msg import image_crop_coordinates, initialization_mask_message

# Data sources
IMAGE_SOURCE = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
               '/Test/Experimentation/Experiment_2024-01-12/Images'
CROP_SOURCE = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
              '/Test/Experimentation/Experiment_2024-01-12/CropCoordinates'
GROUND_TRUTH_SOURCE = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
                      '/Test/Experimentation/Experiment_2024-01-12/GroundTruths'
CSV_RESULTS_DESTINATION = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
                          '/Test/Experimentation/Experiment_2024-01-12/Results/CSV'
IMAGE_RESULTS_DESTINATION = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
                            '/Test/Experimentation/Experiment_2024-01-12/Results/Images'

# Standard Test Factors
#   Accuracy: 0.5
#   DS Rate: 0.5
#   Iteration Count: 2
#   Background Expansion: 1.5
# Define image filter parameters that will be tested
blurring = [False, True]
opening = [False, True]
accuracies_of_initial_mask = [0.5]
down_sampling_rates = [0.5]  # [1, 1 / 2, 1 / 3, 1 / 4, 1 / 5]
iteration_counts = [2]  # [1, 5, 10, 15]
probable_background_expansion_factors = [1.0,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2,2.1,2.2,2.3,2.4,2.5,2.6,2.7,2.8,2.9,3]  # [0.50, 0.75, 1.00, 1.25, 1.50]
capture_image_results = False

# Load the images that will be segmented and their corresponding crop coordinates and ground truth segmentations
images_for_test, image_names_for_test = load_folder_of_image_files(IMAGE_SOURCE, return_image_numbers=True)
image_crop_coordinates_for_test = load_folder_of_numpy_arrays(CROP_SOURCE, convert_results_to_lists=True)
image_ground_truths_for_test = load_folder_of_numpy_arrays(GROUND_TRUTH_SOURCE)

# Capture the time stamp to use for naming all files
file_date_stamp = date_stamp_str(suffix="")

# Otherwise use the parameters listed above
all_test_parameters = None

# If no custom list of parameters is given, generate the list
if all_test_parameters is None:
    # Define a list to store the parameters for each test in
    all_test_parameters = []

    # Define an identifier for each test
    test_number = 0

    # Generate a list of all the test configurations to conduct
    for image_for_test, image_name_for_test, \
        image_crop_coordinate_for_test, image_ground_truth_for_test in zip(images_for_test,
                                                                           image_names_for_test,
                                                                           image_crop_coordinates_for_test,
                                                                           image_ground_truths_for_test):
        for accuracy_of_initial_mask in accuracies_of_initial_mask:
            for down_sampling_rate in down_sampling_rates:
                for iteration_count in iteration_counts:
                    for probable_background_expansion_factor in probable_background_expansion_factors:
                        # Add the parameters to the list
                        all_test_parameters.append([test_number,
                                                    image_for_test, image_name_for_test,
                                                    image_crop_coordinate_for_test, image_ground_truth_for_test,
                                                    accuracy_of_initial_mask, down_sampling_rate, iteration_count,
                                                    probable_background_expansion_factor])

                        # Update the identifier
                        test_number = test_number + 1

    # Calculate how many tests will be conducted
    num_tests = test_number

# Otherwise use the custom list given
else:
    num_tests = len(all_test_parameters)

# Define the number of tests to perform on each set of parameters
num_test_per_parameter_set = 3

# Create the filter_node to be tested
filter_node = ImageFilterNode(filter_type=GRABCUT_FILTER,
                              visualizations_included=[],
                              debug_mode=False, analysis_mode=False)

# Tell the image filter that it is time to filter images
filter_node.filter_images_callback(Bool(True))
filter_node.patient_contact_callback(Bool(True))

# Generate a csv file that will be used to save the data from the test
results_file = open(CSV_RESULTS_DESTINATION + "/Results_" + file_date_stamp + ".csv", mode='w')
results_file_writer = writer(results_file, delimiter=',')
results_file_writer.writerow([TEST_NUM,
                              IMAGE_NUM,
                              INITIAL_MASK_DICE_SCORE,
                              DOWN_SAMPLING_RATE,
                              NUM_ITERATIONS,
                              PROB_BKGD_EXP_FACTOR,
                              SEGMENTATION_TIME_MEDIAN,
                              SEGMENTATION_TIME_AVERAGE,
                              SEGMENTATION_TIME_STD_DEV,
                              DICE_SCORE_MEDIAN,
                              DICE_SCORE_AVERAGE,
                              DICE_SCORE_STD_DEV])

# Create the path to the date stamped folder where the image data will be stored
test_specific_image_results_destination = IMAGE_RESULTS_DESTINATION + "/" + file_date_stamp

# If the path does not exist
if not isdir(test_specific_image_results_destination):

    # Create a folder in the right place
    mkdir(test_specific_image_results_destination)

# Otherwise raise an exception that the folder already exists
else:
    raise Exception("Test-specific destination for image results already exists.")

# For each possible combination
for single_test_parameters in all_test_parameters:

    # Rename all parameters for clarity
    test_number = single_test_parameters[0]
    image_for_test = single_test_parameters[1]
    image_name_for_test = single_test_parameters[2]
    image_crop_coordinate_for_test = single_test_parameters[3]
    image_ground_truth_for_test = single_test_parameters[4]
    accuracy_of_initial_mask = single_test_parameters[5]
    down_sampling_rate = single_test_parameters[6]
    iteration_count = single_test_parameters[7]
    probable_background_expansion_factor = single_test_parameters[8]

    # Send the image crop coordinates to use for cropping the images
    image_crop_coordinates_msg = image_crop_coordinates(first_coordinate_x=image_crop_coordinate_for_test[0][0],
                                                        first_coordinate_y=image_crop_coordinate_for_test[0][1],
                                                        second_coordinate_x=image_crop_coordinate_for_test[1][0],
                                                        second_coordinate_y=image_crop_coordinate_for_test[1][1])
    filter_node.crop_coordinates_callback(image_crop_coordinates_msg)

    # Define two arrays to store the results from each set of tests
    parameter_set_results_time = zeros((num_test_per_parameter_set, 1))
    parameter_set_results_accuracy = zeros((num_test_per_parameter_set, 1))

    # Add the test parameters to the data to save
    row_data = [test_number, image_name_for_test, accuracy_of_initial_mask, down_sampling_rate,
                iteration_count, probable_background_expansion_factor]

    # Complete multiple tests to ensure that the results are valid
    for test_case_number in range(num_test_per_parameter_set):
        # Populate the relevant filter parameters in the image filter
        filter_node.image_filter.down_sampling_rate = down_sampling_rate
        filter_node.image_filter.segmentation_iteration_count = iteration_count
        previous_image_mask_msg = initialization_mask_message(
            previous_image_mask=convert_array_to_image_message(
                build_previous_image_mask_from_ground_truth(image_ground_truth_for_test,
                                                            accuracy_of_initial_mask,
                                                            probable_background_expansion_factor)))
        filter_node.grabcut_initialization_mask_callback(previous_image_mask_msg)

        # Save the previous-image mask as it was loaded into the filter
        previous_image_mask_as_loaded = filter_node.image_filter.previous_image_mask_array

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

        # Resize the resulting mask to the size of the ground truth
        resized_result_mask = resize(filter_node.image_data.image_mask,
                                     dsize=(image_ground_truth_for_test.shape[1],
                                            image_ground_truth_for_test.shape[0]),
                                     # fx=1 / down_sampling_rate, fy=1 / down_sampling_rate,
                                     interpolation=UP_SAMPLING_MODE)

        # Calculate the DICE score of the result of the segmentation
        result_dice_score = calculate_dice_score(image_ground_truth_for_test, resized_result_mask)

        # Add the corresponding results to their arrays
        parameter_set_results_time[test_case_number] = elapsed_time[1]
        parameter_set_results_accuracy[test_case_number] = result_dice_score

    # Calculate the statistics of the data
    time_median = median(parameter_set_results_time)
    time_average = average(parameter_set_results_time)
    time_std_dev = std(parameter_set_results_time)
    accuracy_median = median(parameter_set_results_accuracy)
    accuracy_average = average(parameter_set_results_accuracy)
    accuracy_std_dev = std(parameter_set_results_accuracy)

    # Save the accuracy statistics
    row_data = row_data + [time_median, time_average, time_std_dev,
                           accuracy_median, accuracy_average, accuracy_std_dev]
    results_file_writer.writerow(row_data)

    # Only capture image results if required
    if capture_image_results:

        # Create the prefix for every resulting image
        path_and_prefix = test_specific_image_results_destination + '/' + str(test_number).zfill(5) + '_' + \
            str(image_name_for_test).zfill(5) + '_'

        # Save the down-sampled image
        imwrite(path_and_prefix + 'DownSampled.png', filter_node.image_data.down_sampled_image)

        # Save the previous-image mask
        imwrite(path_and_prefix + 'PreviousImageMask.png', create_mask_display_array(previous_image_mask_as_loaded,
                                                                                     125))

        # Save the mask resulting from the segmentation
        imwrite(path_and_prefix + 'ResultImageMask.png', create_mask_display_array(filter_node.image_data.image_mask,
                                                                                   125))

        # Save the previous-image mask overlay
        imwrite(path_and_prefix + 'PreviousImageMaskOverlay.png',
                create_mask_overlay_array(filter_node.image_data.down_sampled_image,
                                          3, previous_image_mask_as_loaded, COLOR_BGR,
                                          PREV_IMG_MASK))

        # Save the result mask overlay
        imwrite(path_and_prefix + 'ResultMaskOverlay.png',
                create_mask_overlay_array(filter_node.image_data.down_sampled_image,
                                          3, filter_node.image_data.image_mask, COLOR_BGR,
                                          COLORIZED, overlay_color=(0, 45, 0)))

    test_number = test_number + 1

    print('Test ' + str(test_number) + ' of ' + str(num_tests) + ' completed')

# Close the results file
results_file.close()

# for image_name_for_test, image_ground_truth_for_test in zip(image_names_for_test, image_ground_truths_for_test):
#     imwrite(GROUND_TRUTH_SOURCE + '/GroundTruth_' + str(image_name_for_test).zfill(5) + '.png',
#             image_ground_truth_for_test * uint8(125))


# imshow("Image" + str(image_names_for_test[0]), images_for_test[0])
# waitKey(-1)
# imshow("Ground Truth", uint8(image_ground_truths_for_test[0] * 125))
# waitKey(-1)
# imshow("Overlay", create_mask_overlay_array(images_for_test[0], COLOR_BGR,
#                                             resize(image_ground_truths_for_test[0],
#                                                    dsize=(images_for_test[0].shape[1],
#                                                           images_for_test[0].shape[0])),
#                                             COLOR_BGR, COLORIZED, color=(25, 0, 0)))
# waitKey(-1)
# imshow("Previous Image Mask", create_mask_overlay_array(images_for_test[4], COLOR_BGR,
#                                                         resize(build_previous_image_mask_from_ground_truth(
#                                                             image_ground_truths_for_test[4],
#                                                             accuracies_of_initial_mask[0],
#                                                             probable_background_expansion_factors[3]),
#                                                                dsize=(images_for_test[4].shape[1],
#                                                                       images_for_test[4].shape[0])), COLOR_BGR,
#                                                         PREV_IMG_MASK))
# waitKey(-1)

# for image_for_test, image_name_for_test, \
#     image_crop_coordinate_for_test, image_ground_truth_for_test in zip(images_for_test,
#                                                                        image_names_for_test,
#                                                                        image_crop_coordinates_for_test,
#                                                                        image_ground_truths_for_test):
#
#     # Send the image crop coordinates to use for cropping the images
#     image_crop_coordinates_msg = image_crop_coordinates(first_coordinate_x=image_crop_coordinate_for_test[0][0],
#                                                         first_coordinate_y=image_crop_coordinate_for_test[0][1],
#                                                         second_coordinate_x=image_crop_coordinate_for_test[1][0],
#                                                         second_coordinate_y=image_crop_coordinate_for_test[1][1])
#     filter_node.crop_coordinates_callback(image_crop_coordinates_msg)
#
#     for accuracy_of_initial_mask in accuracies_of_initial_mask:
#         for down_sampling_rate in down_sampling_rates:
#             for iteration_count in iteration_counts:
#                 for probable_background_expansion_factor in probable_background_expansion_factors:
#
#                     # Define two arrays to store the results from each set of tests
#                     parameter_set_results_time = zeros((num_test_per_parameter_set, 1))
#                     parameter_set_results_accuracy = zeros((num_test_per_parameter_set, 1))
#
#                     # Add the test parameters to the data to save
#                     row_data = [test_number, image_name_for_test, accuracy_of_initial_mask, down_sampling_rate,
#                                 iteration_count, probable_background_expansion_factor]
#
#                     # Complete multiple tests to ensure that the results are valid
#                     for test_case_number in range(num_test_per_parameter_set):
#                         # Populate the relevant filter parameters in the image filter
#                         filter_node.image_filter.down_sampling_rate = down_sampling_rate
#                         filter_node.image_filter.segmentation_iteration_count = iteration_count
#                         previous_image_mask_msg = initialization_mask_message(
#                             previous_image_mask=convert_array_to_image_message(
#                                 build_previous_image_mask_from_ground_truth(image_ground_truth_for_test,
#                                                                             accuracy_of_initial_mask,
#                                                                             probable_background_expansion_factor)))
#                         filter_node.grabcut_initialization_mask_callback(previous_image_mask_msg)
#
#                         # Create a new image data message
#                         new_image_data_message = ImageData(image_data=cvtColor(copy(image_for_test), COLOR_BGR2GRAY),
#                                                            image_color=COLOR_GRAY).convert_to_message()
#
#                         # Feed the image into the filter_node
#                         filter_node.raw_image_callback(new_image_data_message)
#
#                         # Mark the start time
#                         start_time = time()
#
#                         # Analyze the image
#                         filter_node.main_loop()
#
#                         # Save how long it took to filter the image
#                         elapsed_time = display_process_timer(start_time, "", mode=False, return_time=True)
#
#                         # Calculate the DICE score of the result of the segmentation
#                         result_dice_score = calculate_dice_score(image_ground_truth_for_test,
#                                                                  resize(filter_node.image_data.image_mask,
#                                                                         dsize=(image_ground_truth_for_test.shape[1],
#                                                                                image_ground_truth_for_test.shape[0]),
#                                                                         # fx=1 / down_sampling_rate, fy=1 / down_sampling_rate,
#                                                                         interpolation=UP_SAMPLING_MODE))
#
#                         # Add the corresponding results to their arrays
#                         parameter_set_results_time[test_case_number] = elapsed_time[1]
#                         parameter_set_results_accuracy[test_case_number] = result_dice_score
#
#                     # Calculate the statistics of the data
#                     time_median = median(parameter_set_results_time)
#                     time_average = average(parameter_set_results_time)
#                     time_std_dev = std(parameter_set_results_time)
#                     accuracy_median = median(parameter_set_results_accuracy)
#                     accuracy_average = average(parameter_set_results_accuracy)
#                     accuracy_std_dev = std(parameter_set_results_accuracy)
#
#                     # Save the accuracy statistics
#                     row_data = row_data + [time_median, time_average, time_std_dev,
#                                            accuracy_median, accuracy_average, accuracy_std_dev]
#                     results_file_writer.writerow(row_data)
#
#                     test_number = test_number + 1
#
#                     print('Test ' + str(test_number) + ' of ' + str(num_tests) + ' completed')
#
# # Close the results file
# results_file.close()

# # Plotting axis counter
# ii = 0
#
# # Number of rows and columns for plotting
# NUM_ROWS = 3
# NUM_COLS = 4
#
# # Create a figure to plot the results
# fig, axes = subplots(NUM_ROWS, NUM_COLS)
# fig.suptitle('Image Filter Process', fontsize=36)
# # fig.show()
#

"""# Show the previous image mask as loaded
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(image_for_test)
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title('Original Image\n as Loaded', fontsize=24)
ii = ii + 1"""

# Load the crop coordinates
# crop_coordinates = load(CROP_SOURCE)

# Load the previous image mask that contains the ground truth segmentation
# ground_truth_mask = load(GROUND_TRUTH_SOURCE)

# Show the ground truth mask as it was loaded
"""axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
    cvtColor(ground_truth_mask, COLOR_GRAY2RGB) * array([25, 0, 0]).astype('uint8') +
    cvtColor(new_image_data.cropped_image, COLOR_GRAY2RGB))
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title(
    'Ground Truth\nMask as Loaded', fontsize=24)
ii = ii + 1"""

# Define a variable to store each specific mask
# dice_images = {}

# For each initialization mask score,
"""for accuracy_of_initial_mask in accuracies_of_initial_mask:
    # Generate an initialization mask with a matching score and save it in the dictionary
    dice_images[accuracy_of_initial_mask] = build_previous_image_mask_from_ground_truth(ground_truth_mask,
                                                                                        accuracy_of_initial_mask,
                                                                                        1.0)"""

# Show the ground truth mask as it was loaded
"""axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].imshow(
    cvtColor(dice_images[accuracies_of_initial_mask[0]], COLOR_GRAY2RGB) * array([25, 0, 0]).astype('uint8') +
    cvtColor(new_image_data.cropped_image, COLOR_GRAY2RGB))
axes[int((ii - (ii % NUM_COLS)) / NUM_ROWS)][int(ii % NUM_COLS)].set_title(
    'Previous Image Mask\nas Generated', fontsize=24)
ii = ii + 1"""
