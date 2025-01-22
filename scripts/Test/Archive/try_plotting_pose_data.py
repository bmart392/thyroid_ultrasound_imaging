# Import standard packages
from math import ceil
from numpy import array, zeros, identity
from pymeshfix import clean_from_file
from stl import mesh
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from os.path import abspath

###patch start###
from mpl_toolkits.mplot3d.axis3d import Axis

if not hasattr(Axis, "_get_coord_info_old"):
    def _get_coord_info_new(self, renderer):
        mins, maxs, centers, deltas, tc, highs = self._get_coord_info_old(renderer)
        mins += deltas / 4
        maxs -= deltas / 4
        return mins, maxs, centers, deltas, tc, highs


    Axis._get_coord_info_old = Axis._get_coord_info
    Axis._get_coord_info = _get_coord_info_new
###patch end###


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
pose_source_files = ["/home/ben/thyroid_ultrasound_data/testing_and_validation/BagFileVolumeEstimation"
                     "/LeftLobe/PoseData_LeftLobe/2024-09-19-17_left_lobe_poses.csv",
                     "/home/ben/thyroid_ultrasound_data/testing_and_validation/BagFileVolumeEstimation"
                     "/RightLobe/PoseData_RightLobe/2024-09-19-17_right_lobe_poses.csv",
                     "/home/ben/thyroid_ultrasound_data/testing_and_validation/BagFileVolumeEstimation"
                     "/LeftLobe_PoseComparison/PoseData_LeftLobe/2024-09-19_left_lobe_poses.csv",
                     "/home/ben/thyroid_ultrasound_data/testing_and_validation/BagFileVolumeEstimation"
                     "/RightLobe_PoseComparison/PoseData_RightLobe/2024-09-19_right_lobe_poses.csv", ]



# Set the plot to interactive mode
plt.ion()

# Create a new plot
fig = plt.figure()
visualization_plot = fig.add_subplot(projection='3d')
visualization_plot.set_xlabel('X (m)')
visualization_plot.set_ylabel('Y (m)')
visualization_plot.set_zlabel('Z (m)')
visualization_plot.set_proj_type('ortho')

colors = ['blue', 'blue', 'green', 'green']

for source_file, color in zip(pose_source_files, colors):
    # Read in the pose data from the source file
    poses = read_recorded_data_csv(file_path=source_file,
                                   headings=[TRANSFORM_MATRIX_0, TRANSFORM_MATRIX_1, TRANSFORM_MATRIX_2,
                                             TRANSFORM_MATRIX_3, TRANSFORM_MATRIX_4, TRANSFORM_MATRIX_5,
                                             TRANSFORM_MATRIX_6, TRANSFORM_MATRIX_7, TRANSFORM_MATRIX_8,
                                             TRANSFORM_MATRIX_9, TRANSFORM_MATRIX_10, TRANSFORM_MATRIX_11,
                                             TRANSFORM_MATRIX_12, TRANSFORM_MATRIX_13, TRANSFORM_MATRIX_14,
                                             TRANSFORM_MATRIX_15],
                                   create_combined_stamp=False)

    x_data = []
    y_data = []
    z_data = []

    for destination, source in zip([x_data, y_data, z_data], [poses[TRANSFORM_MATRIX_3], poses[TRANSFORM_MATRIX_7],
                                                              poses[TRANSFORM_MATRIX_11]]):
        for element in source:
            destination.append(float(element))

    visualization_plot.plot(x_data, y_data, z_data, c=color)


print('Done')