"""
File containing the experimentation_FindEquivalentImages script.
"""

# Import standard python packages
from copy import copy
from os import mkdir
from shutil import copyfile
from statistics import mean

# Import custom python packages
from thyroid_ultrasound_support.Functions.generate_ordered_list_of_directory_contents import \
    generate_ordered_list_of_directory_contents
from thyroid_ultrasound_support.Functions.read_recorded_data_csv import read_recorded_data_csv
from thyroid_ultrasound_support.Constants.ExperimentalDataRecordingConstants import TRANSFORM_MATRIX_7, STAMP_SECS, \
    STAMP_NSECS
from thyroid_ultrasound_support.Functions.date_stamp_str import date_stamp_str

# ======================================================================================================================
# Define data locations
# region

# Define the root location of all the data for the lobes
ROOT_LOCATION: str = '/home/ben/thyroid_ultrasound_data/testing_and_validation/BagFileVolumeEstimation'

# Define the location of the data for the left lobe
LEFT_LOBE_IMAGES: str = '/LeftLobe/RawImages_BagFile_2024-09-19_Scan2_LeftLobe'
LEFT_LOBE_POSE: str = '/LeftLobe/PoseData_LeftLobe/2024-09-19-17_left_lobe_poses.csv'
LEFT_LOBE_STAMP_SECONDS: str = '/LeftLobe/PoseData_LeftLobe/2024-09-19-17_left_lobe_timestamp_secs.csv'
LEFT_LOBE_STAMP_NANOSECONDS: str = '/LeftLobe/PoseData_LeftLobe/2024-09-19-17_left_lobe_timestamp_nsecs.csv'

# Define the location of the data for the right lobe
RIGHT_LOBE_IMAGES: str = '/RightLobe/RawImages_BagFile_2024-09-19_Scan2_RightLobe'
RIGHT_LOBE_POSE: str = '/RightLobe/PoseData_RightLobe/2024-09-19-17_right_lobe_poses.csv'
RIGHT_LOBE_STAMP_SECONDS: str = '/RightLobe/PoseData_RightLobe/2024-09-19-17_right_lobe_timestamp_secs.csv'
RIGHT_LOBE_STAMP_NANOSECONDS: str = '/RightLobe/PoseData_RightLobe/2024-09-19-17_right_lobe_timestamp_nsecs.csv'

# Define the location where the paired images are to be stored
PAIRED_LOBE_IMAGE_FOLDER: str = '/PairedLobeImages'

# endregion
# ======================================================================================================================
# Load all data
# region

print('Begin reading in data.')

# Read in the data for the left lobe
ll_poses = read_recorded_data_csv(file_path=ROOT_LOCATION + LEFT_LOBE_POSE,
                                  headings=[TRANSFORM_MATRIX_7], create_combined_stamp=False)
ll_poses = ll_poses[TRANSFORM_MATRIX_7]
ll_poses_sec = read_recorded_data_csv(file_path=ROOT_LOCATION + LEFT_LOBE_STAMP_SECONDS,
                                      headings=[STAMP_SECS], create_combined_stamp=False)
ll_poses_sec = ll_poses_sec[STAMP_SECS]
ll_poses_nsec = read_recorded_data_csv(file_path=ROOT_LOCATION + LEFT_LOBE_STAMP_NANOSECONDS,
                                       headings=[STAMP_NSECS], create_combined_stamp=False)
ll_poses_nsec = ll_poses_nsec[STAMP_NSECS]
ll_poses_stamp = [sec + n_sec * 10 ** -9 for sec, n_sec in zip(ll_poses_sec, ll_poses_nsec)]

# noinspection PyTypeChecker
ll_images_all_data: list = generate_ordered_list_of_directory_contents(ROOT_LOCATION + LEFT_LOBE_IMAGES, (1, 2),
                                                                       return_index_values=True)
ll_images_path = ll_images_all_data[-1]
ll_images_stamp = [sec + n_sec * 10 ** -9 for sec, n_sec in zip(ll_images_all_data[0], ll_images_all_data[1])]

# Read in the data for the right lobe
rl_poses = read_recorded_data_csv(file_path=ROOT_LOCATION + RIGHT_LOBE_POSE,
                                  headings=[TRANSFORM_MATRIX_7], create_combined_stamp=False)
rl_poses = rl_poses[TRANSFORM_MATRIX_7]
rl_poses_sec = read_recorded_data_csv(file_path=ROOT_LOCATION + RIGHT_LOBE_STAMP_SECONDS,
                                      headings=[STAMP_SECS], create_combined_stamp=False)
rl_poses_sec = rl_poses_sec[STAMP_SECS]
rl_poses_nsec = read_recorded_data_csv(file_path=ROOT_LOCATION + RIGHT_LOBE_STAMP_NANOSECONDS,
                                       headings=[STAMP_NSECS], create_combined_stamp=False)
rl_poses_nsec = rl_poses_nsec[STAMP_NSECS]
rl_poses_stamp = [sec + n_sec * 10 ** -9 for sec, n_sec in zip(rl_poses_sec, rl_poses_nsec)]

# noinspection PyTypeChecker
rl_images_all_data: list = generate_ordered_list_of_directory_contents(ROOT_LOCATION + RIGHT_LOBE_IMAGES, (1, 2),
                                                                       return_index_values=True)
rl_images_path = rl_images_all_data[-1]
rl_images_stamp = [sec + n_sec * 10 ** -9 for sec, n_sec in zip(rl_images_all_data[0], rl_images_all_data[1])]

print('All data read in.')

# endregion
# ======================================================================================================================
# Match data points between the two lobes
# region

print('Begin matching poses.')

# Define a list to store the pairs of poses
ll_pose_pair_indices = []
rl_pose_pair_indices = []

# Define variables to store information about the last data point found
prev_smallest_pose_dif = 10 ** 10
prev_index_of_smallest_pose = len(rl_poses)

# Create a dictionary to track the distribution of images saved
distribution_statistics = dict(zip([x * 100 for x in list(range(8))], [copy([]) for x in range(8)]))

# Find each pose in the left lobe set,
for ll_ii, ll_pose in enumerate(ll_poses):

    # Define variables to store data about the most similar point
    smallest_pose_dif = 10 ** 9
    index_of_smallest_pose_dif = None

    # Define a counter to break out of the loop after the smallest difference has been found
    overrun_counter = 0

    # For each pose in the right lobe set,
    for rl_ii, rl_pose in enumerate(rl_poses):

        # Calculate the difference between the two poses
        new_pose_dif = abs(float(ll_pose) - float(rl_pose))

        # Save the new difference and pose index if it is smaller
        if new_pose_dif < smallest_pose_dif:
            smallest_pose_dif = new_pose_dif
            index_of_smallest_pose_dif = rl_ii

        # Otherwise, increment the overrun counter and break out of the loop if necessary
        else:
            overrun_counter = overrun_counter + 1
            if overrun_counter > 5:
                break

    # If the smallest difference is acceptable,
    if smallest_pose_dif < 5 * 10 ** -6:

        # If the same pose was found for this iteration and the previous iteration,
        if prev_index_of_smallest_pose == index_of_smallest_pose_dif:

            # If the new smallest difference is smaller than the previous one,
            if smallest_pose_dif < prev_smallest_pose_dif:

                # Pop out the duplicate pose pair from the list,
                ll_pose_pair_indices.pop(-1)
                rl_pose_pair_indices.pop(-1)

            # Otherwise do not add the current pose pair to the list of matching pairs
            else:
                continue

        # Save the new pose pair
        ll_pose_pair_indices.append(ll_ii)
        rl_pose_pair_indices.append(index_of_smallest_pose_dif)

        # Save the new data as the data for the previous iteration
        prev_index_of_smallest_pose = index_of_smallest_pose_dif
        prev_smallest_pose_dif = smallest_pose_dif

        # Update the statistics
        insertion_index = index_of_smallest_pose_dif - (index_of_smallest_pose_dif % 100)
        distribution_statistics[insertion_index].append(smallest_pose_dif)

print('Pose matching is complete.')

# endregion
# ======================================================================================================================
# Display information about the matched data points
# region

print('---')
print('The distribution of images along the path is:')
previous_key = 0
for key in distribution_statistics.keys():
    print("{previous_key:3d}".format(previous_key=previous_key) + ' -> ' +
          str(key + 100) + ': ' + '{images:2d}'.format(images=len(distribution_statistics[key])) + ' images | ' +
          'Avg. Pose Error (m): {error:2e}'.format(error=mean(distribution_statistics[key])))
    previous_key = key + 101
print('Total pairs found: ' + str(len(ll_pose_pair_indices)))
print('---')

# endregion
# ======================================================================================================================
# Find the matching images for each pose
# region

print('Begin matching images.')

# Define a list to store the pairs of images
ll_image_pair_indices = []
rl_image_pair_indices = []

ll_image_pair_time_differences = []
rl_image_pair_time_differences = []

for pose_pair_indices, poses_stamp, \
    images_stamp, image_pair_indices, \
    image_pair_time_differences, images_path in zip([ll_pose_pair_indices, rl_pose_pair_indices],
                                                    [ll_poses_stamp, rl_poses_stamp],
                                                    [ll_images_stamp, rl_images_stamp],
                                                    [ll_image_pair_indices, rl_image_pair_indices],
                                                    [ll_image_pair_time_differences, rl_image_pair_time_differences],
                                                    [ll_images_path, rl_images_path], ):

    # Define variables to store information about the last data point found
    prev_smallest_time_dif = 10 ** 10
    prev_index_of_smallest_time_dif = len(pose_pair_indices)

    # For each pose index,
    for pose_pair_index in pose_pair_indices:

        # Pull out the time stamp of the pose
        pose_pair_stamp = poses_stamp[pose_pair_index]

        # Define variables to store data about the most similar point
        smallest_time_dif = 10 ** 9
        index_of_smallest_time_dif = None

        # Define a counter to break out of the loop after the smallest difference has been found
        overrun_counter = 0

        # For each image timestamp,
        for single_image_stamp_index, single_image_stamp in enumerate(images_stamp):

            # Calculate the difference between the pose pair timestamp and the image timestamp
            new_time_dif = abs(float(pose_pair_stamp) - float(single_image_stamp))

            # Save the new difference and image timestamp index if it is smaller
            if new_time_dif < smallest_time_dif:
                smallest_time_dif = new_time_dif
                index_of_smallest_time_dif = single_image_stamp_index

            # Otherwise, increment the overrun counter and break out of the loop if necessary
            else:
                overrun_counter = overrun_counter + 1
                if overrun_counter > 5:
                    break

        # If the smallest difference is acceptable,
        if smallest_time_dif < 1. * 10 ** -2:

            # If the same pose was found for this iteration and the previous iteration,
            if prev_index_of_smallest_time_dif == index_of_smallest_time_dif:

                # If the new smallest difference is smaller than the previous one,
                if smallest_time_dif < prev_smallest_time_dif:

                    # Pop out the duplicate pose pair from the list,
                    image_pair_indices[-1] = None
                    image_pair_time_differences[-1] = None

                # Otherwise add a None placeholder to the list instead
                else:

                    image_pair_indices.append(None)
                    image_pair_time_differences.append(None)
                    continue

            # Save the new pose pair
            image_pair_indices.append(index_of_smallest_time_dif)
            image_pair_time_differences.append(smallest_time_dif)

            # Save the new data as the data for the previous iteration
            prev_index_of_smallest_time_dif = index_of_smallest_time_dif
            prev_smallest_time_dif = smallest_time_dif

        # Otherwise add a None placeholder to the list instead
        else:

            image_pair_indices.append(None)
            image_pair_time_differences.append(None)

# Define a list to store the image pairs
paired_image_paths = []

# Define variables to store the average time difference
ll_paired_image_time_differences = []
rl_paired_image_time_differences = []

for ll_image_pair_time_difference, rl_image_pair_time_difference, \
    ll_image_pair_index, rl_image_pair_index in zip(ll_image_pair_time_differences, rl_image_pair_time_differences,
                                                    ll_image_pair_indices, rl_image_pair_indices):

    if ll_image_pair_time_difference is not None and rl_image_pair_time_difference is not None:
        paired_image_paths.append((ll_images_path[ll_image_pair_index],
                                   rl_images_path[rl_image_pair_index]))
        ll_paired_image_time_differences.append(ll_image_pair_time_difference)
        rl_paired_image_time_differences.append(rl_image_pair_time_difference)

print('Image matching is complete.')

# endregion
# ======================================================================================================================
# Display information about the matched data points
# region

print('---')
print(str(len(paired_image_paths)) + ' image pairs were found.')
print('Left Lobe Avg. Time Error (s):  ' + str('{error:2e}'.format(error=mean(ll_paired_image_time_differences))))
print('Right Lobe Avg. Time Error (s): ' + str('{error:2e}'.format(error=mean(rl_paired_image_time_differences))))
print('---')

# endregion
# ======================================================================================================================
# Create folders containing the paired images and the label for each slide
# region

print('Begin copying the images.')

# Create the folder to contain the image pairs from this script execution
containing_folder = ROOT_LOCATION + PAIRED_LOBE_IMAGE_FOLDER + '/ImagePairs' + date_stamp_str(prefix='_', suffix='')
mkdir(containing_folder)

# For each image pair,
for pair_index, image_paths in enumerate(paired_image_paths):

    # Convert the pair index to a string of constant length
    pair_index_as_str = str(pair_index + 1)
    if len(pair_index_as_str) == 1:
        pair_index_as_str = '0' + pair_index_as_str

    # Create a new folder to hold the pair of images
    temp_image_folder = containing_folder + '/Pair_' + pair_index_as_str
    mkdir(temp_image_folder)

    # Capture just the file names of the images
    file_names = [path.split('/')[-1] for path in image_paths]

    # Copy the images into the folder
    for temp_path, file_name in zip(image_paths, file_names):
        copyfile(temp_path, temp_image_folder + '/' + file_name)

    # Create a text document that contains label for each slide
    simple_data_file = open(temp_image_folder + '/Pair_' + pair_index_as_str + '_Label.txt', 'w')
    simple_data_file.write('Left Lobe: ' + file_names[0][:-4] + ' | Right Lobe: ' + file_names[1][:-4])
    simple_data_file.close()

print('All images copied.')

# endregion
# ======================================================================================================================

print('Script complete.')
