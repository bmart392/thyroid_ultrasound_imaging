# Import standard packages
from math import ceil, floor
from numpy import array, zeros, identity, mean, eye
from pymeshfix import clean_from_file
from stl import mesh
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from os.path import abspath
from copy import deepcopy

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

# TODO - HIGH - Some points are being left out, need to ensure all points are connected. ALso need to compare closest paired points for multiple points at once

# Import custom packages
from thyroid_ultrasound_imaging_support.RegisteredData.RegisteredData import RegisteredData, ImageData
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ZX_ALIGNED
from thyroid_ultrasound_imaging_support.RegisteredData.load_folder_of_saved_registered_data import \
    load_folder_of_saved_registered_data
from thyroid_ultrasound_imaging_support.VolumeGeneration.create_mesh_triangles import create_mesh_triangles
from thyroid_ultrasound_imaging_support.VolumeGeneration.create_mesh_triangles_v2 import create_mesh_triangles_v2
from thyroid_ultrasound_imaging_support.VolumeGeneration.plot_transformation import plot_transformation
from thyroid_ultrasound_imaging_support.VolumeGeneration.display_mesh_information import display_mesh_information
from thyroid_ultrasound_imaging_support.VolumeGeneration.combine_meshes import combine_meshes
from thyroid_ultrasound_imaging_support.ImageFilter.remove_isolated_pixes import remove_isolated_pixes
from thyroid_ultrasound_imaging_support.VolumeGeneration.find_external_points_v2 import find_external_points_v2
from thyroid_ultrasound_imaging_support.VolumeGeneration.set_axes_equal import set_axes_equal

# ============================================================================
# Correct method to combine meshes - Tested with Rebuilt Lobes
# both_lobes_combined = combine_meshes(
#     mesh_1='/home/ben/thyroid_ultrasound_data/testing_and_validation/BagFileVolumeEstimation'
#            '/RightLobe/stlFiles_RightLobe/RightLobe_repaired.stl',
#     mesh_2='/home/ben/thyroid_ultrasound_data/testing_and_validation/BagFileVolumeEstimation'
#            '/LeftLobe/stlFiles_LeftLobe/LeftLobe_repaired.stl',
#     save_result_mesh='/home/ben/Desktop/both_lobes_repaired.stl',
#     display_mesh_data='Both Lobes')

mesh_name = 'RightLobe'
# Read in the data
# list_of_registered_data = \
#     load_folder_of_saved_registered_data('/home/ben/thyroid_ultrasound_data/testing_and_validation'
#                                          '/BagFileVolumeEstimation/RightLobe/RegisteredData_RightLobe')
right_list_of_registered_data: list = \
    load_folder_of_saved_registered_data('/home/ben/thyroid_ultrasound_data/testing_and_validation'
                                         '/BagFileVolumeEstimation/RightLobe/RegisteredData_RightLobe_Updated')
left_list_of_registered_data: list = \
    load_folder_of_saved_registered_data('/home/ben/thyroid_ultrasound_data/testing_and_validation'
                                         '/BagFileVolumeEstimation/LeftLobe/RegisteredData_LeftLobe_Updated')
# load_folder_of_saved_registered_data('/home/ben/thyroid_ultrasound_data/testing_and_validation'
#                                      '/BagFileVolumeEstimation/LeftLobe/RegisteredData_LeftLobe')
# load_folder_of_saved_registered_data('/home/ben/thyroid_ultrasound_data/testing_and_validation/'
#                                      'registered_data/Exam_2024-03-17_17-09-13-590628')
# load_folder_of_saved_registered_data('/home/ben/thyroid_ultrasound_data/testing_and_validation/'
#                                      'VolumeGenerationExperiment/VolumeData/'
#                                      'VolumeData_2024-04-10_15-28-34-603222_2024-04-11_08-20-47-828371')


# Create a variable to store the point cloud data
this_point_cloud = []

# Create a variable to define the x location of each point cloud
placement_index = 0

# Set the plot to interactive mode
plt.ion()

# Create a new plot
fig = plt.figure()
visualization_plot = fig.add_subplot(projection='3d')
visualization_plot.set_xlabel('X (m)')
visualization_plot.set_ylabel('Y (m)')
visualization_plot.set_zlabel('Z (m)')
visualization_plot.set_proj_type('ortho')

# Calculate
robot_position_zeroing_transformation = zeros((4, 4))
robot_position_zeroing_transformation[0:3, 3] = (left_list_of_registered_data[0].robot_pose.robot_pose[0:3, 3] +
                                                 right_list_of_registered_data[0].robot_pose.robot_pose[0:3, 3]) / 2

# Pop out the first registered data object from each set
left_list_of_registered_data.pop(0)
right_list_of_registered_data.pop(0)

# For the data for each lobe,
for list_of_registered_data in [left_list_of_registered_data]:  # , right_list_of_registered_data]:

    # Define lists to hold data about each contour
    all_centroids = []
    all_external_contour_points = []
    all_contour_points = []

    # For each registered data,
    for contour_count in range(len(list_of_registered_data)):

        # Pull out the image data for convenience
        image_data = list_of_registered_data[contour_count].image_data

        # Pull out the robot pose and zero it out for convenience
        robot_pose_at_image_capture_time = list_of_registered_data[contour_count].robot_pose.robot_pose - \
                                           robot_position_zeroing_transformation

        # Plot the robot pose
        plot_transformation(robot_pose_at_image_capture_time, visualization_plot)

        # Remove any isolated pixels in the image data
        remove_isolated_pixes(image_data.post_processed_mask)

        # Generate the contours from the image mask and find the centroid of each contour
        image_data.generate_contours_in_image()
        image_data.calculate_image_centroids()

        # Calculate the slicing step for the contour
        slice_step = image_data.contours_in_image[0].shape[0] / 100
        if slice_step > 1:
            slice_step = floor(slice_step)
        else:
            slice_step = ceil(slice_step)

        # Plot the contour for debugging purposes
        if contour_count == -1:
            temp_plt = plt.figure()
            temp_x = image_data.contours_in_image[0][::slice_step][:, 0].tolist()
            temp_y = image_data.contours_in_image[0][::slice_step][:, 1].tolist()
            temp_axes = temp_plt.add_subplot()
            temp_axes.scatter(temp_x, temp_y)
            plt.show()
        else:
            temp_axes = None

        # Calculate the external points on the contour
        indices_of_external_points_on_contour = find_external_points_v2(image_data.contours_in_image[0][::slice_step].tolist())

        # Generate a transformed list of points for the largest contour
        transformed_contours, transformed_vectors, transformed_centroids = image_data.generate_transformed_contours(
            transformation=robot_pose_at_image_capture_time,
            imaging_depth=image_data.imaging_depth / 100,
        )

        # Select only the first contour and centroid
        if len(transformed_contours) > 1:
            transformed_contours = [transformed_contours[0]]
        if len(transformed_centroids) > 1:
            transformed_centroids = [transformed_centroids[0]]

        # If the contour and centroid has length,
        if len(transformed_contours) > 0 or len(transformed_centroids) > 0:

            # Select only a portion of the full contour
            abridged_transformed_contours = transformed_contours[0][::slice_step]

            # # Generate lists of the internal and external points of the contour
            # internal_points_array = []
            # external_points_array = []
            # for ii in range(abridged_transformed_contours.shape[0]):
            #     if ii in indices_of_external_points_on_contour:
            #         external_points_array.append(abridged_transformed_contours[ii])
            #     else:
            #         internal_points_array.append(abridged_transformed_contours[ii])
            #
            # # If there are internal points, plot them
            # if len(internal_points_array) > 0:
            #     internal_points_array = array(internal_points_array)
            #     visualization_plot.scatter3D(array(internal_points_array)[:, 0],
            #                                  array(internal_points_array)[:, 1],
            #                                  array(internal_points_array)[:, 2],
            #                                  c='black', s=10)
            #
            # # If there are external points, plot them
            # if len(external_points_array) > 0:
            #     external_points_array = array(external_points_array)
            #     visualization_plot.scatter3D(array(external_points_array)[:, 0],
            #                                  array(external_points_array)[:, 1],
            #                                  array(external_points_array)[:, 2],
            #                                  c='red', s=25)

            # Always plot the centroid
            visualization_plot.scatter3D(array(transformed_centroids)[0, :, 0],
                                         array(transformed_centroids)[0, :, 1],
                                         array(transformed_centroids)[0, :, 2],
                                         c='black', s=75
                                         )

            # Adjust the axes of the plot so they are equal
            set_axes_equal(visualization_plot)

            # Save the points of the contour, the centroid, and the list of external points
            this_point_cloud.append([tuple(point) for point in abridged_transformed_contours])
            all_centroids.append(tuple(transformed_centroids[0][0]))
            all_external_contour_points.append(indices_of_external_points_on_contour)

# Calculate the triangles in the point cloud
point_cloud_triangles = create_mesh_triangles_v2(this_point_cloud, visualization_plot, all_centroids,
                                                 all_external_contour_points)

print("Number of faces: " + str(len(point_cloud_triangles)))

# Convert the triangles to an array
point_cloud_triangles_as_arrays = array(point_cloud_triangles)

# Create a mesh and add the triangles
volume_mesh = mesh.Mesh(zeros(point_cloud_triangles_as_arrays.shape[0], dtype=mesh.Mesh.dtype))
volume_mesh.vectors = point_cloud_triangles_as_arrays

volume_mesh.save('/home/ben/Desktop/' + mesh_name + '_unrepaired.stl')

unrepaired_mesh = mesh.Mesh.from_file('/home/ben/Desktop/' + mesh_name + '_unrepaired.stl')
clean_from_file('/home/ben/Desktop/' + mesh_name + '_unrepaired.stl',
                '/home/ben/Desktop/' + mesh_name + '_repaired.stl')
repaired_mesh = mesh.Mesh.from_file('/home/ben/Desktop/' + mesh_name + '_repaired.stl')

display_mesh_information('Repaired Mesh', existing_mesh=repaired_mesh)
# volume, cog, inertia = repaired_mesh.get_mass_properties()
# print("Volume (mm^3)                           = {0}".format(volume / 10 ** -9))
# print("Volume (mL)                             = {0}".format(volume * 10 ** 6))
# print("Position of the center of gravity (COG) = {0}".format(cog))
# print("Inertia matrix at expressed at the COG  = {0}".format(inertia[0, :]))
# print("                                          {0}".format(inertia[1, :]))
# print("                                          {0}".format(inertia[2, :]))

"""# Turn off interactive plotting
plt.ioff()

# Create a new plot
figure = plt.figure()
axes = mplot3d.Axes3D(figure)"""

# Load the STL files and add the vectors to the plot
collection = mplot3d.art3d.Poly3DCollection(volume_mesh.vectors)
collection.set(facecolor=[1, 1, 1], alpha=1., linewidth=1, edgecolor=[0, 0, 0])
visualization_plot.add_collection3d(collection)

# Auto-scale to the mesh size
# scale = volume_mesh.points.flatten()
# visualization_plot.auto_scale_xyz(scale, scale, scale)

# Show the plot to the screen
plt.show()
print("hi")
