# Import standard ROS packages
from geometry_msgs.msg import WrenchStamped

# Import standard python packages
from cv2 import imshow, waitKey
from numpy import array
from os import mkdir

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_support.Functions.date_stamp_str import date_stamp_str

# Import custom python packages
from thyroid_ultrasound_support.Functions.generate_ordered_list_of_directory_contents import \
    generate_ordered_list_of_directory_contents
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.Visualization.create_mask_display_array import create_mask_display_array
from thyroid_ultrasound_imaging_support.Visualization.create_mask_overlay_array import create_mask_overlay_array, \
    COLORIZED
from thyroid_ultrasound_imaging_support.RegisteredData.update_dict_with_pose import update_dict_with_pose
from thyroid_ultrasound_imaging_support.RegisteredData.find_closest_key_value import find_closest_key_value
from thyroid_ultrasound_support.Functions.read_recorded_data_csv import read_recorded_data_csv
from thyroid_ultrasound_support.Constants.ExperimentalDataRecordingConstants import *
from thyroid_ultrasound_imaging_support.RegisteredData.RegisteredData import RegisteredData, RobotForce, \
    SAVE_OBJECT
from ImagePositionRegistrationNode import OBJECT, FOLDER_OF_REGISTERED_DATA_PREFIX
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_saved_image_data import \
    load_folder_of_saved_image_data

# Define which experiment data to look at
image_data_directory = "/home/ben/thyroid_ultrasound_data/testing_and_validation/BagFileVolumeEstimation" \
                       "/LeftLobe/ImageData_LeftLobe_Updated"

pose_data_root_directory = "/home/ben/thyroid_ultrasound_data/testing_and_validation/BagFileVolumeEstimation" \
                           "/LeftLobe/PoseData_LeftLobe"
pose_source_file = "/2024-09-19-17_left_lobe_poses.csv"
timestamp_sec_source_file = "/2024-09-19-17_left_lobe_timestamp_secs.csv"
timestamp_nsec_source_file = "/2024-09-19-17_left_lobe_timestamp_nsecs.csv"

init_node("reviewCollectedData")

# -----------
# Read in pose data
# region

# Read in the pose data from the source file
poses = read_recorded_data_csv(file_path=pose_data_root_directory + pose_source_file,
                               headings=[TRANSFORM_MATRIX_0, TRANSFORM_MATRIX_1, TRANSFORM_MATRIX_2, TRANSFORM_MATRIX_3,
                                         TRANSFORM_MATRIX_4, TRANSFORM_MATRIX_5, TRANSFORM_MATRIX_6, TRANSFORM_MATRIX_7,
                                         TRANSFORM_MATRIX_8, TRANSFORM_MATRIX_9, TRANSFORM_MATRIX_10,
                                         TRANSFORM_MATRIX_11,
                                         TRANSFORM_MATRIX_12, TRANSFORM_MATRIX_13, TRANSFORM_MATRIX_14,
                                         TRANSFORM_MATRIX_15],
                               create_combined_stamp=False)
timestamps_sec = read_recorded_data_csv(file_path=pose_data_root_directory + timestamp_sec_source_file,
                                        headings=[STAMP_SECS], create_combined_stamp=False)
timestamps_nsec = read_recorded_data_csv(file_path=pose_data_root_directory + timestamp_nsec_source_file,
                                         headings=[STAMP_NSECS], create_combined_stamp=False)

# Saved pose data
pose_data = {}

# For each piece of saved data about the robot pose,
for ii in range(len(timestamps_sec[STAMP_SECS])):
    # Add a new entry to the dictionary with the corresponding data
    update_dict_with_pose(pose_as_matrix=array([[poses[TRANSFORM_MATRIX_0][ii],
                                                 poses[TRANSFORM_MATRIX_1][ii],
                                                 poses[TRANSFORM_MATRIX_2][ii],
                                                 poses[TRANSFORM_MATRIX_3][ii]],
                                                [poses[TRANSFORM_MATRIX_4][ii],
                                                 poses[TRANSFORM_MATRIX_5][ii],
                                                 poses[TRANSFORM_MATRIX_6][ii],
                                                 poses[TRANSFORM_MATRIX_7][ii]],
                                                [poses[TRANSFORM_MATRIX_8][ii],
                                                 poses[TRANSFORM_MATRIX_9][ii],
                                                 poses[TRANSFORM_MATRIX_10][ii],
                                                 poses[TRANSFORM_MATRIX_11][ii]],
                                                [poses[TRANSFORM_MATRIX_12][ii],
                                                 poses[TRANSFORM_MATRIX_13][ii],
                                                 poses[TRANSFORM_MATRIX_14][ii],
                                                 poses[TRANSFORM_MATRIX_15][ii]]]),
                          message_time_secs=timestamps_sec[STAMP_SECS][ii],
                          message_time_nsecs=timestamps_nsec[STAMP_NSECS][ii], dict_to_update=pose_data)

# endregion
# -----------
# Read in images
# region

# Create an ordered list of all image data
# paths_to_image_data_objects = list(generate_ordered_list_of_directory_contents(image_data_directory,
#                                                                                (1, 2)))

# Load the saved image data objects
image_data_objects = load_folder_of_saved_image_data(image_data_directory)

# # Define a list to store the recreated image data objects
# image_data_objects_sorted_by_file_name = []
#
# # Load the saved image data objects,
# for image_data_path in paths_to_image_data_objects[-1]:
#     # Rebuild the ImageData object at the given location
#     image_data_objects_sorted_by_file_name.append(ImageData(image_data_location=image_data_path))

# endregion
# -----------
# Create a dummy force message to use to populate the data
# region
dummy_message = WrenchStamped()
dummy_message.wrench.force.x = 0
dummy_message.wrench.force.y = 0
dummy_message.wrench.force.z = 3
dummy_message.wrench.torque.x = 0
dummy_message.wrench.torque.y = 0
dummy_message.wrench.torque.z = 0

# endregion
# -----------
# Create the folder where the data will be saved
# region
save_data_location = '/home/ben/thyroid_ultrasound_data/testing_and_validation/registered_data' \
                     + FOLDER_OF_REGISTERED_DATA_PREFIX + date_stamp_str(prefix='_', suffix='')
mkdir(save_data_location)

# endregion
# -----------
# Match the image data to the pose data and save it as a regular set of registered data

# Origin pose
origin_pose = None

for i, new_object in enumerate(image_data_objects):

    if sum(new_object.image_mask.shape) > 0:

        closest_pose_sec, closest_pose_nsec = find_closest_key_value(
            seconds_key_to_search_for=new_object.image_capture_time.secs,
            nanoseconds_key_to_search_for=new_object.image_capture_time.nsecs,
            dict_to_search_in=pose_data,
            additional_offset_steps=1)

        try:
            distance_travelled = origin_pose[0:3, 3] - pose_data[closest_pose_sec][closest_pose_nsec][
                                                           OBJECT].robot_pose[0:3, 3]
        except TypeError:
            origin_pose = pose_data[closest_pose_sec][closest_pose_nsec][OBJECT].robot_pose
            distance_travelled = array([0, 2, 0])

        print(distance_travelled)

        if distance_travelled[1] > 0.001:

            # Update the dummy message header
            dummy_message.header.stamp.secs = closest_pose_sec
            dummy_message.header.stamp.nsecs = closest_pose_nsec

            # Display the image briefly
            imshow("Original Image", create_mask_overlay_array(base_image=new_object.down_sampled_image,
                                                               overlay_mask=new_object.image_mask,
                                                               overlay_method=COLORIZED,
                                                               overlay_color=(0, 0, 25)))

            # imshow("Mask", create_mask_display_array(new_object.image_mask))
            # if (i % 20) == 0:
            #     waitKey(-1)
            # else:

            image_approval = waitKey(-1)

            # If the image is approved,
            if image_approval == 13:  # 13 = Enter Key

                origin_pose = pose_data[closest_pose_sec][closest_pose_nsec][OBJECT].robot_pose

                # Publish the pose and the image
                new_registered_data = RegisteredData(image_data_object=new_object,
                                                     robot_pose=pose_data[closest_pose_sec]
                                                     [closest_pose_nsec][OBJECT],
                                                     robot_force=RobotForce(source_message=dummy_message),
                                                     )

                # Save the new registered data object
                new_registered_data.save_load(action=SAVE_OBJECT,
                                              path_to_file_location=save_data_location)

                # Remove both the pose and the force from the selection of stored data
                try:
                    pose_data[closest_pose_sec].pop(closest_pose_nsec)
                except KeyError:
                    pass

            # If the image is rejected,
            elif image_approval == 27:  # 27 = Esc Key
                print(str(new_object.image_capture_time.secs) + "," + str(new_object.image_capture_time.nsecs))

print("hi")
