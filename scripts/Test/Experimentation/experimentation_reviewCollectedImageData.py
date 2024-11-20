# Import standard ROS packages
from geometry_msgs.msg import WrenchStamped

# Import standard python packages
from cv2 import imshow, waitKey

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_support.Functions.date_stamp_str import date_stamp_str

# Import custom python packages
from thyroid_ultrasound_support.Functions.generate_ordered_list_of_directory_contents import \
    generate_ordered_list_of_directory_contents
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.Visualization.create_mask_display_array import create_mask_display_array
from thyroid_ultrasound_imaging_support.RegisteredData.update_dict_with_pose import update_dict_with_pose
from thyroid_ultrasound_imaging_support.RegisteredData.find_closest_key_value import find_closest_key_value
from thyroid_ultrasound_support.Functions.read_recorded_data_csv import read_recorded_data_csv
from thyroid_ultrasound_robot_control_support.Helpers.calc_transformation_from_rpy import calc_transformation_from_rpy
from thyroid_ultrasound_support.Constants.ExperimentalDataRecordingConstants import *
from thyroid_ultrasound_imaging_support.RegisteredData.RegisteredData import RegisteredData, RobotForce, \
    SAVE_OBJECT
from ImagePositionRegistrationNode import OBJECT, FOLDER_OF_REGISTERED_DATA_PREFIX

# Define which experiment data to look at
root_directory = "/home/ben/thyroid_ultrasound_data/experimentation"
image_source_directory = "/2024-11-07_20-02-34-762794_experiment/ImageData_2024-11-07_20-02-34-762794"
pose_source_file = "/2024-11-07_20-02-34-762794_experiment/Pose_2024-03-14_14-50-40-145753.csv"

init_node("reviewCollectedData")

# -----------
# Read in pose data
# region

# Read in the pose data from the source file
data = read_recorded_data_csv(file_path=root_directory + pose_source_file,
                              headings=[MESSAGE_ID, STAMP_SECS, STAMP_NSECS,
                                        POSE_X, POSE_Y, POSE_Z,
                                        POSE_ROLL, POSE_PITCH, POSE_YAW,
                                        WAYPOINT_REACHED],
                              sort_heading=MESSAGE_ID)

# Saved pose data
pose_data = {}

# For each piece of saved data about the robot pose,
for sec, n_sec, \
    x, y, z, \
    roll, pitch, yaw in zip(data[STAMP_SECS], data[STAMP_NSECS],
                            data[POSE_X], data[POSE_Y], data[POSE_Z],
                            data[POSE_ROLL], data[POSE_PITCH], data[POSE_YAW]):
    # Add a new entry to the dictionary with the corresponding data
    update_dict_with_pose(pose_as_matrix=calc_transformation_from_rpy(xyz=(x, y, z), rpy=(-roll, pitch, yaw)),
                          message_time_secs=sec, message_time_nsecs=n_sec, dict_to_update=pose_data)

# endregion
# -----------
# Read in images
# region

# Create an ordered list of all image data
paths_to_image_data_objects = generate_ordered_list_of_directory_contents(root_directory + image_source_directory,
                                                                          (1, 2))

# Define a list to store the recreated image data objects
image_data_objects_sorted_by_file_name = []

# Load the saved image data objects,
for image_data_path in paths_to_image_data_objects[-1]:
    # Rebuild the ImageData object at the given location
    image_data_objects_sorted_by_file_name.append(ImageData(image_data_location=image_data_path))

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
# endregion
# -----------
# Match the image data to the pose data and save it as a regular set of registered data
for i, new_object in enumerate(image_data_objects_sorted_by_file_name):

    if sum(new_object.image_mask.shape) > 0:

        closest_pose_nsec = find_closest_key_value(seconds_key_to_search_for=new_object.image_capture_time.secs,
                                                   nanoseconds_key_to_search_for=new_object.image_capture_time.nsecs,
                                                   dict_to_search_in=pose_data)

        # Update the dummy message header
        dummy_message.header.stamp.secs = new_object.image_capture_time.secs
        dummy_message.header.stamp.nsecs = closest_pose_nsec

        # Display the image briefly
        imshow("Original Image", new_object.original_image)

        imshow("Mask", create_mask_display_array(new_object.image_mask))
        # if (i % 20) == 0:
        #     waitKey(-1)
        # else:
        waitKey(10)

        # Publish the pose and the image
        new_registered_data = RegisteredData(image_data_object=new_object,
                                             robot_pose=pose_data[new_object.image_capture_time.secs]
                                             [closest_pose_nsec][OBJECT],
                                             robot_force=RobotForce(source_message=dummy_message),
                                             )

        # Save the new registered data object
        new_registered_data.save_load(action=SAVE_OBJECT,
                                      path_to_file_location=save_data_location)

        # Remove both the pose and the force from the selection of stored data
        try:
            pose_data[new_object.image_capture_time.secs].pop(closest_pose_nsec)
        except KeyError:
            pass

print("hi")
