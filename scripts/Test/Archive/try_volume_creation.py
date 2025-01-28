# Import standard packages
from math import ceil, floor
from numpy import array, zeros, identity, mean, eye, append
from pymeshfix import clean_from_file
from stl import mesh
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d.axis3d import Axis
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from os.path import abspath
from copy import deepcopy

# Import custom packages
from thyroid_ultrasound_imaging_support.RegisteredData.load_folder_of_saved_registered_data import \
    load_folder_of_saved_registered_data
from thyroid_ultrasound_imaging_support.VolumeGeneration.create_mesh_triangles_v2 import create_mesh_triangles_v2
from thyroid_ultrasound_imaging_support.VolumeGeneration.plot_transformation import plot_transformation
from thyroid_ultrasound_imaging_support.VolumeGeneration.display_mesh_information import display_mesh_information
from thyroid_ultrasound_imaging_support.ImageFilter.remove_isolated_pixes import remove_isolated_pixes
from thyroid_ultrasound_imaging_support.VolumeGeneration.find_external_points_v2 import find_external_points_v2
from thyroid_ultrasound_imaging_support.VolumeGeneration.set_axes_equal import set_axes_equal

# TODO - HIGH - Some points are being left out, need to ensure all points are connected. ALso need to compare closest paired points for multiple points at once

# ======================================================================================================================
# Patch the 3D plot function
# region
if not hasattr(Axis, "_get_coord_info_old"):
    def _get_coord_info_new(self, renderer):
        mins, maxs, centers, deltas, tc, highs = self._get_coord_info_old(renderer)
        mins += deltas / 4
        maxs -= deltas / 4
        return mins, maxs, centers, deltas, tc, highs


    Axis._get_coord_info_old = Axis._get_coord_info
    Axis._get_coord_info = _get_coord_info_new

# endregion
# ======================================================================================================================
# Define options for the test
# region

# Lobe data selection options
RIGHT_LOBE_ONLY: str = 'right_lobe_only'
LEFT_LOBE_ONLY: str = 'left_lobe_only'
BOTH_LOBES: str = 'both_lobes'

# 2D progress plot locations
ORIGINAL_CONTOUR: tuple = (0, 0)
CLEANED_CONTOUR: tuple = (0, 1)
REDUCED_CONTOUR: tuple = (1, 0)
INTERNAL_EXTERNAL_POINTS: tuple = (1, 1)

# endregion
# ======================================================================================================================
# Define the parameters of the test

# Define the number of data points to remove from each set of registered data
num_data_points_to_remove: int = 3

# Select if the progress of the test should be plotted in 3D
plot_3D_progress: bool = True

# Select if the progress for preparing the contours should be plotted in 2D
plot_2D_progress: bool = False
slices_to_monitor: list = [0, 5, 15, 20]

# Select which lobe data will be used
LOBE_OPTION: str = RIGHT_LOBE_ONLY
mesh_name = 'RightLobe'

# endregion
# ======================================================================================================================
# Read in the data for the lobes
# region

# Read in the data for the right lobe
if LOBE_OPTION == RIGHT_LOBE_ONLY or LOBE_OPTION == BOTH_LOBES:
    # noinspection PyTypeChecker
    right_lobe_data: list = \
        load_folder_of_saved_registered_data('/home/ben/thyroid_ultrasound_data/testing_and_validation'
                                             '/BagFileVolumeEstimation/RightLobe/RegisteredData_RightLobe_Updated')

    # Pop out the required number of data points
    for i in [0] * num_data_points_to_remove:
        right_lobe_data.pop(0)
    right_lobe_data.pop(-1)

    # Pop out the first robot pose for later
    right_lobe_first_pose = right_lobe_data[0].robot_pose.robot_pose[0:3, 3]

else:
    right_lobe_data = []
    right_lobe_first_pose = zeros(3)

# Read in the data for the left lobe
if LOBE_OPTION == LEFT_LOBE_ONLY or LOBE_OPTION == BOTH_LOBES:
    # noinspection PyTypeChecker
    left_lobe_data: list = \
        load_folder_of_saved_registered_data('/home/ben/thyroid_ultrasound_data/testing_and_validation'
                                             '/BagFileVolumeEstimation/LeftLobe/RegisteredData_LeftLobe_Updated')

    # Pop out the required number of data points
    for i in [0] * num_data_points_to_remove:
        left_lobe_data.pop(0)
    left_lobe_data.pop(-1)

    # Pop out the first robot pose for later
    left_lobe_first_pose = left_lobe_data[0].robot_pose.robot_pose[0:3, 3]

else:
    left_lobe_data = []
    left_lobe_first_pose = zeros(3)

# Calculate the transformation to zero the lobe positions
robot_position_zeroing_transformation = zeros((4, 4))
robot_position_zeroing_transformation[0:3, 3] = (left_lobe_first_pose + right_lobe_first_pose) / 2

# endregion
# ======================================================================================================================
# Setup the plot for showing the progress in 3D
# region

# Set the plot to interactive mode
plt.ion()

# Create a new plot for visualizing the 3D results
if plot_3D_progress:
    fig = plt.figure()
    visualization_plot_3d = fig.add_subplot(projection='3d')
    visualization_plot_3d.set_xlabel('X (m)')
    visualization_plot_3d.set_ylabel('Y (m)')
    visualization_plot_3d.set_zlabel('Z (m)')
    visualization_plot_3d.set_proj_type('ortho')
else:
    visualization_plot_3d = None

# endregion
# ======================================================================================================================
# Define additional variables
# region

# Create a variable to store the point cloud data
this_point_cloud = []

# endregion
# ======================================================================================================================


# For the data for each lobe,
for current_lobe_data, current_lobe_name in [(left_lobe_data, 'Left Lobe'), (right_lobe_data, 'Right Lobe')]:

    # Define lists to hold the data about the current lobe
    current_lobe_centroids = []
    current_lobe_external_contour_points = []
    current_lobe_contour_points = []
    current_lobe_triangles = []

    # For each registered data in the current lobe,
    for contour_count in range(len(current_lobe_data)):

        # Pull out the image data for convenience
        image_data = current_lobe_data[contour_count].image_data

        # Pull out the robot pose and zero it out for convenience
        robot_pose_at_image_capture_time = current_lobe_data[contour_count].robot_pose.robot_pose - \
                                           robot_position_zeroing_transformation

        # Plot the robot pose
        plot_transformation(robot_pose_at_image_capture_time, visualization_plot_3d)

        # Create the plot to display the progress of the
        if plot_2D_progress and contour_count in slices_to_monitor:

            # Create the overall figure for the progress plots
            fig_2D, visualization_axes_2D = plt.subplots(2, 2)
            fig_2D.suptitle(current_lobe_name + ' - Slice ' + str(contour_count), fontsize=16)

            # Generate the contours from the original image mask
            image_data.generate_contours_in_image()

            # Plot the original contour and add the title
            visualization_axes_2D[ORIGINAL_CONTOUR].plot(append(image_data.contours_in_image[0][:, 0],
                                                                image_data.contours_in_image[0][0, 0]),
                                                         append(image_data.contours_in_image[0][:, 1],
                                                                image_data.contours_in_image[0][0, 1]))
            visualization_axes_2D[ORIGINAL_CONTOUR].set_title('Original Contour - ' +
                                                              str(len(image_data.contours_in_image[0][:, 0])) +
                                                              ' Points')

        else:
            fig_2D = None
            visualization_axes_2D = None

        # Remove any isolated pixels in the image data
        remove_isolated_pixes(image_data.post_processed_mask)

        # Generate the contours from the image mask and find the centroid of each contour
        image_data.generate_contours_in_image()
        image_data.calculate_image_centroids()

        # Plot the contour with all isolated pixels removed and add the title
        if fig_2D is not None:
            visualization_axes_2D[CLEANED_CONTOUR].plot(append(image_data.contours_in_image[0][:, 0],
                                                               image_data.contours_in_image[0][0, 0]),
                                                        append(image_data.contours_in_image[0][:, 1],
                                                               image_data.contours_in_image[0][0, 1]))
            visualization_axes_2D[CLEANED_CONTOUR].set_title('Cleaned Contour - ' +
                                                             str(len(image_data.contours_in_image[0][:, 0])) +
                                                             ' Points')

        # Calculate the slicing step for the contour
        slice_step = image_data.contours_in_image[0].shape[0] / 100
        if slice_step > 1:
            slice_step = floor(slice_step)
        else:
            slice_step = ceil(slice_step)

        # Select only a portion of the full contour
        abridged_contour = image_data.contours_in_image[0][::slice_step]

        # Plot the abridged version of the contour and add the title
        if fig_2D is not None:
            visualization_axes_2D[REDUCED_CONTOUR].plot(append(abridged_contour[:, 0],
                                                               abridged_contour[0, 0]),
                                                        append(abridged_contour[:, 1],
                                                               abridged_contour[0, 1]))
            visualization_axes_2D[REDUCED_CONTOUR].set_title('Reduced Contour - ' +
                                                             str(len(abridged_contour[:, 0])) +
                                                             ' Points')

        if contour_count == 300:
            print('Break')
            fig_temp, visualization_axes_external_points = plt.subplots(1)
        else:
            visualization_axes_external_points = None

        # Calculate the external points on the contour
        indices_of_external_points_on_contour = find_external_points_v2(abridged_contour.tolist(),
                                                                        visualization_plot=visualization_axes_external_points)

        # Plot the internal and external points of the contour and add the title
        if fig_2D is not None:
            # region
            # Generate lists of the internal and external points of the contour
            internal_points_array_x = []
            internal_points_array_y = []
            external_points_array_x = []
            external_points_array_y = []

            for i in range(len(abridged_contour)):
                if i in indices_of_external_points_on_contour:
                    external_points_array_x.append(abridged_contour[i, 0])
                    external_points_array_y.append(abridged_contour[i, 1])
                else:
                    internal_points_array_x.append(abridged_contour[i, 0])
                    internal_points_array_y.append(abridged_contour[i, 1])

            visualization_axes_2D[INTERNAL_EXTERNAL_POINTS].scatter(external_points_array_x,
                                                                    external_points_array_y,
                                                                    c='green', s=10)
            visualization_axes_2D[INTERNAL_EXTERNAL_POINTS].scatter(internal_points_array_x,
                                                                    internal_points_array_y,
                                                                    c='red', s=10)
            visualization_axes_2D[INTERNAL_EXTERNAL_POINTS].set_title('Internal vs External Points - ' +
                                                                      str(len(external_points_array_x)) +
                                                                      ' External Points')

            # endregion

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

            # Generate lists of the internal and external points of the contour
            internal_points_array = []
            external_points_array = []
            for ii in range(abridged_transformed_contours.shape[0]):
                if ii in indices_of_external_points_on_contour:
                    external_points_array.append(abridged_transformed_contours[ii])
                else:
                    internal_points_array.append(abridged_transformed_contours[ii])

            # If there are internal points, plot them
            if len(internal_points_array) > 0:
                internal_points_array = array(internal_points_array)
                visualization_plot_3d.scatter3D(array(internal_points_array)[:, 0],
                                                array(internal_points_array)[:, 1],
                                                array(internal_points_array)[:, 2],
                                                c='black', s=10)

            # If there are external points, plot them
            if len(external_points_array) > 0:
                external_points_array = array(external_points_array)
                visualization_plot_3d.scatter3D(array(external_points_array)[:, 0],
                                                array(external_points_array)[:, 1],
                                                array(external_points_array)[:, 2],
                                                c='red', s=25)

            # Always plot the centroid
            visualization_plot_3d.scatter3D(array(transformed_centroids)[0, :, 0],
                                            array(transformed_centroids)[0, :, 1],
                                            array(transformed_centroids)[0, :, 2],
                                            c='black', s=75
                                            )

            # Adjust the axes of the plot so they are equal
            set_axes_equal(visualization_plot_3d)

            # Save the points of the contour, the centroid, and the list of external points
            current_lobe_contour_points.append([tuple(point) for point in abridged_transformed_contours])
            current_lobe_centroids.append(tuple(transformed_centroids[0][0]))
            current_lobe_external_contour_points.append(indices_of_external_points_on_contour)

    # TODO - High - The only change I should philosophically need to make is that when it looks to see how many points
    #  are between two points it looks at all the points but it tries to find mathing pairs within the external points
    #  first
    # Calculate the triangles in the point cloud
    current_lobe_triangles = create_mesh_triangles_v2(current_lobe_contour_points, visualization_plot_3d,
                                                      current_lobe_centroids,
                                                      current_lobe_external_contour_points)

    print("Number of faces: " + str(len(current_lobe_triangles)))

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
visualization_plot_3d.add_collection3d(collection)

# Auto-scale to the mesh size
# scale = volume_mesh.points.flatten()
# visualization_plot_3d.auto_scale_xyz(scale, scale, scale)

# Show the plot to the screen
plt.show()
print("hi")
