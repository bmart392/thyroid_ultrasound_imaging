from csv import DictWriter
from os import mkdir

from cv2 import imshow, waitKey, morphologyEx, MORPH_OPEN, MORPH_CLOSE, MORPH_DILATE, MORPH_ERODE, imread, VideoWriter, \
    VideoWriter_fourcc
from numpy import newaxis, std, uint8, ones, median, frombuffer, reshape, zeros, array, mean, seterr, linspace
from time import perf_counter
from copy import deepcopy
from matplotlib.pyplot import show, plot, subplots, ion, ioff, gcf

from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageFilter.FilterConstants import COLOR_GRAY
from thyroid_ultrasound_imaging_support.ImageFilter.SegmentationError import SegmentationError
from thyroid_ultrasound_support.Constants.SharedConstants import GROWTH_PHASE, REST_PHASE
from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilterGrabCut import ImageFilterGrabCut
from ImageBasedUserInput import ImageBasedUserInput, GENERATE_CROP_FROM_TEMPLATE, GENERATE_INITIALIZATION
from thyroid_ultrasound_support.Functions.date_stamp_str import date_stamp_str
from thyroid_ultrasound_support.Functions.generate_ordered_list_of_directory_contents import \
    generate_ordered_list_of_directory_contents
from thyroid_ultrasound_imaging_support.Visualization.create_mask_overlay_array import create_mask_overlay_array, \
    COLORIZED
from thyroid_ultrasound_imaging_support.Visualization.stitch_image_arrays import stitch_image_arrays
from thyroid_ultrasound_imaging_support.ImageData.ImageData import CROPPED_IMAGE, COLORIZED_IMAGE, DOWN_SAMPLED_IMAGE, \
    PRE_PROCESSED_IMAGE, IMAGE_MASK, POST_PROCESSED_MASK, SURE_FOREGROUND_MASK, SURE_BACKGROUND_MASK, \
    PROBABLE_FOREGROUND_MASK, SEGMENTATION_INITIALIZATION_MASK

seterr(all='raise')


# ----------------------------------------------------------------------------------------------------------------------
# Local Functions
# region


def collect_creation_times(data_dictionary, data_type, process_start_time):
    data_dictionary[data_type].append(perf_counter() - process_start_time)
    return perf_counter()


def remove_outliers(data):
    num_data_to_average = 50
    for ii in range(num_data_to_average + 1, len(data)):
        subset = array(data[ii - num_data_to_average - 1: ii - 1])
        average_value = mean(subset)
        standard_deviation = std(subset)
        if (average_value + (4 * standard_deviation)) < data[ii] < (average_value - (4 * standard_deviation)):
            data[ii] = (data[ii - 1] + data[ii + 1]) / 2

        if (average_value * 1.5) < data[ii] < (average_value * 0.5):
            print('hi')


# endregion
# ----------------------------------------------------------------------------------------------------------------------
# Define location in which to save data
# region
save_data_path = '/home/ben/thyroid_ultrasound_data/testing_and_validation/image_segmentation_tests'

# endregion
# ----------------------------------------------------------------------------------------------------------------------
# Path to images - N/A
# region

# Define the path to the test images
path_to_images = '/home/ben/thyroid_ultrasound_data/experimentation/' \
                 'VolumeGeneration/2024-04-10_15-24-00-724108_experiment/' \
                 'RawImages_2024-04-10_15-24-00-724108'

# Image options
run_images_in_reverse = False
image_start_point = 0

# endregion
# ----------------------------------------------------------------------------------------------------------------------
# Script options
# region

# Options for selecting the segmentation parameters
SINGLE_OPTION = 'single'
MULTI_OPTION = 'multi'

segmentation_setting_selection = SINGLE_OPTION  # Select how the parameters to test will be selected
visualization_on = True  # Display the progress of the segmentation
save_diagnostic_data = False  # Saves data about each trial

# endregion
# ----------------------------------------------------------------------------------------------------------------------
# Define the segmentation settings using the multi option
# region

# Down sampling rates
down_sampling_rates = [0.35, 0.30, 0.25]

# FG Creation DICE Scores
fg_creation_dice_scores = [0.3, 0.5, 0.8]

# BG Creation DICE Scores
bg_creation_dice_scores = [1.2, 1.4, 1.7]

# Foreground kernel sizes
fg_kernel_sizes = [(3, 3), (5, 5), (7, 7)]

# Background kernel sizes
bg_kernel_sizes = [(3, 3), (5, 5), (7, 7)]

# endregion
# ----------------------------------------------------------------------------------------------------------------------
# Define the segmentation settings using the single option
# region

# Order of parameters is:
# -----------------------
# 1.    down_sampling_rate
# 2.    fg_creation_dice_score
# 3.    bg_creation_dice_score
# 4.    fg_kernel_size
# 5.    bg_kernel_size

single_options = [[0.3, 0.8, 1.2, (3, 3), (5, 5)],
                  [0.3, 0.8, 1.25, (3, 3), (5, 5)],
                  [0.3, 0.8, 1.3, (3, 3), (5, 5)],
                  [0.3, 0.8, 1.4, (3, 3), (5, 5)],
                  ]

# endregion
# ----------------------------------------------------------------------------------------------------------------------
# Build the segmentation options to use in the trials
# region

settings_for_each_trial = []

if segmentation_setting_selection == SINGLE_OPTION:
    settings_for_each_trial = single_options

elif segmentation_setting_selection == MULTI_OPTION:
    for down_sampling_rate in down_sampling_rates:
        for fg_creation_dice_score in fg_creation_dice_scores:
            for bg_creation_dice_score in bg_creation_dice_scores:
                for fg_kernel_size in fg_kernel_sizes:
                    for bg_kernel_size in bg_kernel_sizes:
                        settings_for_each_trial.append([down_sampling_rate, fg_creation_dice_score,
                                                        bg_creation_dice_score, fg_kernel_size, bg_kernel_size])

else:
    raise Exception('Unrecognized segmentation setting selection')

# Calculate the number of trials to complete
total_num_trials = len(settings_for_each_trial)

print('Trials to complete: ' + str(total_num_trials))

# endregion
# ----------------------------------------------------------------------------------------------------------------------
# Load the images - N/A
# region

try:
    images = [imread(path) for path in generate_ordered_list_of_directory_contents(
        path_to_images, sort_indices=(0, 1))]
except Exception:
    try:
        images = [imread(path) for path in generate_ordered_list_of_directory_contents(
            path_to_images, sort_indices=tuple([0]))]
    except Exception:
        raise Exception('Image files not named as expected.')

# Reverse the order of the images if needed
if run_images_in_reverse:
    images.reverse()

# Set the start point for the images as necessary
images = images[image_start_point:]

# endregion
# ----------------------------------------------------------------------------------------------------------------------
# Create constants to store data from the experiment
# region

TRIAL_NUM = 'Trial #'
DS_RATE = 'Down-sampling Rate'
FG_SIZE = 'FG Kernel\nSize (Px)'
BG_SIZE = 'BG Kernel\nSize (Px)'
FG_SCORE = 'FG Creation\nDICE Score'
BG_SCORE = 'BG Creation\nDICE Score'
FIS_CUTOFF = 'First Image\nSection Cutoff'
LIS_CUTOFF = 'Last Image\nSection Cutoff'
MEDIAN_SEG_RATE = 'Median Segmentation\nRate (Hz)'
STD_DEV_SEG_RATE = 'Std. Dev. of\nSegmentation Rate (Hz)'
NUM_POINTS = '# of Points'
PERCENT_STABILITY = '% Stability'
IMAGE_MASK_MEDIAN = 'Image Mask\nMedian (ms)'
IMAGE_MASK_STD = 'Image Mask\nStd. Dev. (ms)'
IMAGE_MASK_CHANGE = 'Image Mask\nChange (ms)'
POST_PROCESSED_MASK_MEDIAN = 'Post-processed\nMask Median (ms)'
POST_PROCESSED_MASK_STD = 'Post-processed\nMask Std. Dev. (ms)'
POST_PROCESSED_MASK_CHANGE = 'Post-processed\nMask Change (ms)'
SURE_FOREGROUND_MASK_MEDIAN = 'Sure Foreground\nMask Median (ms)'
SURE_FOREGROUND_MASK_STD = 'Sure Foreground\nMask Std. Dev. (ms)'
SURE_FOREGROUND_MASK_CHANGE = 'Sure Foreground\nMask Change (ms)'
SURE_BACKGROUND_MASK_MEDIAN = 'Sure Background\nMask Median (ms)'
SURE_BACKGROUND_MASK_STD = 'Sure Background\nMask Std. Dev. (ms)'
SURE_BACKGROUND_MASK_CHANGE = 'Sure Background\nMask Change (ms)'

# endregion
# ----------------------------------------------------------------------------------------------------------------------
# Create structure to save data from the experiment
# region

if save_diagnostic_data:
    # Generate a new data stamp
    date_stamp = date_stamp_str(suffix="")

    # Create a new folder structure for this set of data
    if save_data_path[-1] == '/':
        save_data_path = save_data_path[:-1]
    current_folder_path = save_data_path + '/' + date_stamp + '_experiment' + '/'
    mkdir(current_folder_path)

    # Create a folder to store the video from each test
    video_folder_path = current_folder_path + 'Videos/'
    mkdir(video_folder_path)

    # Save the path to the image set used for this test
    image_path_file = open(current_folder_path + 'SourceImagePath.txt', mode='w')
    image_path_file.write(path_to_images)
    image_path_file.close()

    # Create the CSV document in which to save the data
    data_file = open(current_folder_path + 'Data_' + date_stamp + ".csv", mode='w')
    data_file_writer = DictWriter(data_file, delimiter=',',
                                  fieldnames=[
                                      TRIAL_NUM, DS_RATE, FG_SIZE, BG_SIZE, FG_SCORE, BG_SCORE, FIS_CUTOFF, LIS_CUTOFF,
                                      MEDIAN_SEG_RATE, STD_DEV_SEG_RATE, NUM_POINTS, PERCENT_STABILITY,
                                      IMAGE_MASK_MEDIAN, IMAGE_MASK_STD, IMAGE_MASK_CHANGE,
                                      POST_PROCESSED_MASK_MEDIAN, POST_PROCESSED_MASK_STD, POST_PROCESSED_MASK_CHANGE,
                                      SURE_FOREGROUND_MASK_MEDIAN, SURE_FOREGROUND_MASK_STD,
                                      SURE_FOREGROUND_MASK_CHANGE,
                                      SURE_BACKGROUND_MASK_MEDIAN, SURE_BACKGROUND_MASK_STD,
                                      SURE_BACKGROUND_MASK_CHANGE,
                                  ])
    data_file_writer.writeheader()

# endregion
# ----------------------------------------------------------------------------------------------------------------------
# Prepare the image filter and user input module - N/A
# region

# Create the user input node
ui_node = ImageBasedUserInput()

# Select the crop coordinates
ui_node.actions.append(GENERATE_CROP_FROM_TEMPLATE)
crop_coordinates_msg = ui_node.main_loop()

# Create the filter and apply the crop coordinates
image_filter = ImageFilterGrabCut()
image_filter.image_crop_coordinates = [
    [crop_coordinates_msg.first_coordinate_x, crop_coordinates_msg.first_coordinate_y],
    [crop_coordinates_msg.second_coordinate_x, crop_coordinates_msg.second_coordinate_y]
]

# Create an image data object
first_image_object = ImageData(image_data=images[0][:, :, 0], image_color=COLOR_GRAY)

# Select the ROI and create initialization mask
image_filter.crop_image(first_image_object)
image_filter.colorize_image(first_image_object)
ui_node.image_for_mask_and_threshold = first_image_object
ui_node.actions.append(GENERATE_INITIALIZATION)
initialization_mask_msg = ui_node.main_loop(local_image_data=first_image_object)

# Convert the initialization mask from message form to array form
initialization_mask = frombuffer(initialization_mask_msg.previous_image_mask.data, dtype=uint8)
initialization_mask = reshape(initialization_mask, (initialization_mask_msg.previous_image_mask.height,
                                                    initialization_mask_msg.previous_image_mask.width))

# endregion
# ----------------------------------------------------------------------------------------------------------------------
# Define counter for trials
trial_counter = 0

for settings in settings_for_each_trial:
    down_sampling_rate = settings[0]
    fg_creation_dice_score = settings[1]
    bg_creation_dice_score = settings[2]
    fg_kernel_size = settings[3]
    bg_kernel_size = settings[4]

    # --------------------------------------------------------------------------------------------------
    # Set the filter parameters
    # region
    image_filter.down_sampling_rate = down_sampling_rate
    image_filter.sure_foreground_creation_dice_score = fg_creation_dice_score
    image_filter.sure_background_creation_dice_score = bg_creation_dice_score
    image_filter.sure_foreground_creation_kernel_size = fg_kernel_size
    image_filter.sure_background_creation_kernel_size = bg_kernel_size
    image_filter.sure_foreground_creation_iterations = None
    image_filter.sure_background_creation_iterations = None

    # Reset the list of previous image masks
    image_filter.previous_image_masks = []

    # Update the image crop coordinates
    image_filter.image_crop_coordinates = [
        [crop_coordinates_msg.first_coordinate_x, crop_coordinates_msg.first_coordinate_y],
        [crop_coordinates_msg.second_coordinate_x, crop_coordinates_msg.second_coordinate_y]
    ]

    # Save the mask to the image filter
    image_filter.update_previous_image_mask(initialization_mask)

    # Set the phase of the filter
    image_filter.segmentation_phase = GROWTH_PHASE

    # endregion
    # --------------------------------------------------------------------------------------------------
    # Initialize diagnostic variables - N/A
    # region
    segmentation_rates = []
    mask_sizes = []
    num_stable = 0.0
    creation_times = {CROPPED_IMAGE: [], COLORIZED_IMAGE: [], DOWN_SAMPLED_IMAGE: [],
                      PRE_PROCESSED_IMAGE: [],
                      IMAGE_MASK: [], POST_PROCESSED_MASK: [], SURE_FOREGROUND_MASK: [],
                      SURE_BACKGROUND_MASK: [],
                      PROBABLE_FOREGROUND_MASK: [], SEGMENTATION_INITIALIZATION_MASK: []}

    # endregion
    # --------------------------------------------------------------------------------------------------
    # Define parameters for generating the video
    # region
    if save_diagnostic_data:
        fourcc = VideoWriter_fourcc(*'mp4v')
        first_image_flag = True
        video_name = video_folder_path + 'Trial_' + str(trial_counter) + '.avi'
        video_frame_rate = int(30)
        this_video = None
    # endregion

    try:
        for index, raw_image in enumerate(images):

            # ------------------------------------------------------------------------------------------
            # Create the image data object - N/A
            # region

            image_data_object = ImageData(image_data=raw_image[:, :, 0], image_color=COLOR_GRAY)

            # endregion
            # ------------------------------------------------------------------------------------------
            # Filter the image - N/A
            # region

            timing_variable = perf_counter()
            image_filter.crop_image(image_data_object)
            timing_variable = collect_creation_times(creation_times, CROPPED_IMAGE, timing_variable)
            image_filter.colorize_image(image_data_object)
            timing_variable = collect_creation_times(creation_times, COLORIZED_IMAGE, timing_variable)
            image_filter.down_sample_image(image_data_object)
            timing_variable = collect_creation_times(creation_times, DOWN_SAMPLED_IMAGE,
                                                     timing_variable)
            image_filter.pre_process_image(image_data_object)
            timing_variable = collect_creation_times(creation_times, PRE_PROCESSED_IMAGE,
                                                     timing_variable)
            image_filter.create_image_mask(image_data_object)
            timing_variable = collect_creation_times(creation_times, IMAGE_MASK, timing_variable)
            diagnostic_data = image_filter.post_process_image_mask(image_data_object)
            timing_variable = collect_creation_times(creation_times, POST_PROCESSED_MASK,
                                                     timing_variable)
            image_filter.create_sure_foreground_mask(image_data_object)
            timing_variable = collect_creation_times(creation_times, SURE_FOREGROUND_MASK,
                                                     timing_variable)
            image_filter.create_sure_background_mask(image_data_object)
            timing_variable = collect_creation_times(creation_times, SURE_BACKGROUND_MASK,
                                                     timing_variable)
            image_filter.create_probable_foreground_mask(image_data_object)
            timing_variable = collect_creation_times(creation_times, PROBABLE_FOREGROUND_MASK,
                                                     timing_variable)
            image_filter.create_previous_image_mask(image_data_object)
            timing_variable = collect_creation_times(creation_times, SEGMENTATION_INITIALIZATION_MASK,
                                                     timing_variable)
            image_data_object.generate_contours_in_image()
            timing_variable

            # endregion
            # ------------------------------------------------------------------------------------------
            # Determine if the image segmentation is stable
            # region

            if image_filter.has_segmentation_stabilized():
                num_stable = num_stable + 1

            # endregion
            # ------------------------------------------------------------------------------------------
            # Record data about this iteration
            # region

            # Calculate the net segmentation rate
            net_segmentation_time = 0.0
            for key in creation_times.keys():
                net_segmentation_time = net_segmentation_time + creation_times[key][-1]
            this_segmentation_rate = round(1 / net_segmentation_time, 1)

            # Record the segmentation rate and mask size
            segmentation_rates.append(this_segmentation_rate)
            mask_sizes.append(
                (sum(sum(image_data_object.image_mask)) / (image_data_object.ds_image_size_x *
                                                           image_data_object.ds_image_size_y)) * 100)

            # Add to the video
            if save_diagnostic_data and first_image_flag:
                this_video = VideoWriter(video_name, fourcc, video_frame_rate,
                                         (image_data_object.ds_image_size_x,
                                          image_data_object.ds_image_size_y))
                first_image_flag = False

            # Display the images
            mask_overlay = create_mask_overlay_array(image_data_object.down_sampled_image,
                                                     image_data_object.image_mask,
                                                     overlay_method=COLORIZED,
                                                     overlay_color=(0, 25, 0))

            # Show the simulation progress if desired
            if visualization_on:
                imshow('Mask Overlay', mask_overlay)
                waitKey(1)

            if save_diagnostic_data:
                # Save the image
                this_video.write(mask_overlay)

            # endregion
    except SegmentationError:
        pass

    # Release the video
    if save_diagnostic_data and this_video is not None:
        this_video.release()

    if not save_diagnostic_data:
        print('=' * 100)
        print('Trial #: ' + str(trial_counter) + ' | DS Rate: ' + str(down_sampling_rate) + ' | FG Creation Score: ' +
              str(fg_creation_dice_score) + ' | BG Creation Score: ' + str(bg_creation_dice_score) +
              ' | FG Kernel Size: ' + str(fg_kernel_size) + ' | BG Kernel Size: ' + str(bg_kernel_size))
        print('-' * 100)

    # --------------------------------------------------------------------------------------------------
    # Save data about the trial
    # region

    # Convert the segmentation rates list to an array
    segmentation_rates = array(segmentation_rates)

    # Set the bounds of the first and last section of images
    first_section = int(len(segmentation_rates) * 0.5)
    last_section = int(len(segmentation_rates) * 0.95)

    row_data = {
        TRIAL_NUM: trial_counter, DS_RATE: down_sampling_rate, FG_SIZE: fg_kernel_size[0], BG_SIZE: bg_kernel_size[0],
        FG_SCORE: fg_creation_dice_score, BG_SCORE: bg_creation_dice_score,
        MEDIAN_SEG_RATE: 0,
        STD_DEV_SEG_RATE: 0,
        NUM_POINTS: len(segmentation_rates), FIS_CUTOFF: first_section, LIS_CUTOFF: last_section,
        PERCENT_STABILITY: 0,
        IMAGE_MASK_MEDIAN: 0, IMAGE_MASK_STD: 0, IMAGE_MASK_CHANGE: 0,
        POST_PROCESSED_MASK_MEDIAN: 0, POST_PROCESSED_MASK_STD: 0, POST_PROCESSED_MASK_CHANGE: 0,
        SURE_FOREGROUND_MASK_MEDIAN: 0, SURE_FOREGROUND_MASK_STD: 0, SURE_FOREGROUND_MASK_CHANGE: 0,
        SURE_BACKGROUND_MASK_MEDIAN: 0, SURE_BACKGROUND_MASK_STD: 0, SURE_BACKGROUND_MASK_CHANGE: 0
    }

    if len(segmentation_rates) > 2:

        row_data[MEDIAN_SEG_RATE] = round(median(segmentation_rates), 2)
        row_data[STD_DEV_SEG_RATE] = round(std(segmentation_rates), 4)
        row_data[PERCENT_STABILITY] = round((num_stable / len(segmentation_rates)) * 100, 1)

        if not save_diagnostic_data:
            print('Median Segmentation Rate (Hz): ' + str(row_data[MEDIAN_SEG_RATE]) +
                  ' | Std. Dev. of Segmentation Rate (Hz): ' + str(row_data[STD_DEV_SEG_RATE]) +
                  ' | % Stability: ' + str(row_data[PERCENT_STABILITY]))
            print('-' * 100)
            segmentation_rate_string = 'Individual Segmentation Rates (Hz) |'
            for ii in linspace(0, 1, 5):
                ii = int(ii * (len(segmentation_rates) - 1))
                segmentation_rate_string = segmentation_rate_string + ' Image #' + str(ii) + \
                                           ': ' + format(round(segmentation_rates[ii], 2), '06.2f') + ' |'
            print(segmentation_rate_string)
            print('-' * 100)

        keys_to_plot = [IMAGE_MASK, POST_PROCESSED_MASK, SURE_FOREGROUND_MASK, SURE_BACKGROUND_MASK]

        for key, data_fields in zip(keys_to_plot,
                                    [[IMAGE_MASK_MEDIAN, IMAGE_MASK_STD, IMAGE_MASK_CHANGE],
                                     [POST_PROCESSED_MASK_MEDIAN, POST_PROCESSED_MASK_STD,
                                      POST_PROCESSED_MASK_CHANGE],
                                     [SURE_FOREGROUND_MASK_MEDIAN, SURE_FOREGROUND_MASK_STD,
                                      SURE_FOREGROUND_MASK_CHANGE],
                                     [SURE_BACKGROUND_MASK_MEDIAN, SURE_BACKGROUND_MASK_STD,
                                      SURE_BACKGROUND_MASK_CHANGE]]):
            remove_outliers(creation_times[key])
            temp_array = array(creation_times[key])

            row_data[data_fields[0]] = round(median(temp_array) * 1000, 4)
            row_data[data_fields[1]] = round(std(temp_array) * 1000, 4)
            row_data[data_fields[2]] = round((median(temp_array[last_section:]) -
                                              median(temp_array[:first_section])) * 1000, 4)

            if not save_diagnostic_data:
                print('Median (ms): ' + format(row_data[data_fields[0]], '07.4f') +
                      ' | Std. Dev. (ms): ' + format(row_data[data_fields[1]], '.4f') +
                      ' | Change in Time (ms): ' + format(row_data[data_fields[2]], '+08.4f') +
                      ' | ' + key)

        # Set success message
        success_message = 'successfully'

    else:
        success_message = 'unsuccessfully'

    if save_diagnostic_data:
        # Write in the data to the CSV file
        data_file_writer.writerow(row_data)

        print('Trial ' + str(trial_counter + 1) + ' of ' + str(total_num_trials) +
              ' ' + success_message + ' completed.')

    trial_counter = trial_counter + 1

# Close the CSV file once all data is written
if save_diagnostic_data:
    data_file.close()
