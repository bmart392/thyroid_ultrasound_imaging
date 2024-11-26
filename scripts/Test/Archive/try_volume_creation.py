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

# Import custom packages
from thyroid_ultrasound_imaging_support.RegisteredData.RegisteredData import RegisteredData, ImageData
from thyroid_ultrasound_imaging_support.RegisteredData.load_folder_of_saved_registered_data import \
    load_folder_of_saved_registered_data
from thyroid_ultrasound_imaging_support.VolumeGeneration.create_mesh_triangles import create_mesh_triangles
from thyroid_ultrasound_imaging_support.VolumeGeneration.plot_transformation import plot_transformation
from thyroid_ultrasound_imaging_support.VolumeGeneration.display_mesh_information import display_mesh_information
from thyroid_ultrasound_imaging_support.VolumeGeneration.combine_meshes import combine_meshes

# ============================================================================
# Correct method to combine meshes - Tested with Basic Shapes
# rod_cube_mesh = combine_meshes(mesh_1='/home/ben/Desktop/test_rod.stl',
#                                mesh_2='/home/ben/Desktop/test_square.stl',
#                                save_result_mesh='/home/ben/Desktop/test_rod_cube_combined_repaired.stl',
#                                display_mesh_data='Rod + Cube')
# rod_weird_mesh = combine_meshes(mesh_1='/home/ben/Desktop/test_rod.stl',
#                                 mesh_2='/home/ben/Desktop/test_weird.stl',
#                                 save_result_mesh='/home/ben/Desktop/test_rod_weird_combined_repaired.stl',
#                                 display_mesh_data='Rod + Weird')
# cube_weird_mesh = combine_meshes(mesh_1='/home/ben/Desktop/test_square.stl',
#                                  mesh_2='/home/ben/Desktop/test_weird.stl',
#                                  save_result_mesh='/home/ben/Desktop/test_cube_weird_combined_repaired.stl',
#                                  display_mesh_data='Cube + Weird')

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
right_list_of_registered_data = \
    load_folder_of_saved_registered_data('/home/ben/thyroid_ultrasound_data/testing_and_validation'
                                         '/BagFileVolumeEstimation/RightLobe/RegisteredData_RightLobe_Updated')
left_list_of_registered_data = \
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
visualization_plot.set_xlabel('X Label')
visualization_plot.set_ylabel('Y Label')
visualization_plot.set_zlabel('Z Label')
visualization_plot.set_proj_type('ortho')

for list_of_registered_data in [left_list_of_registered_data]:

    first_pose = list_of_registered_data[1].robot_pose.robot_pose

    first_pose_position = zeros((4, 4))
    first_pose_position[0:3, 3] = first_pose[0:3, 3]

    # For each registered data, pull out the contour and save it to the point cloud
    for registered_data in list_of_registered_data[1:3]:
        registered_data: RegisteredData

        # Pull out the corresponding data
        image_data = registered_data.image_data
        robot_pose_at_image_capture_time = identity(4) + (registered_data.robot_pose.robot_pose - first_pose)
        # robot_pose_at_image_capture_time = registered_data.robot_pose.robot_pose - first_pose

        # Plot the transformation
        plot_transformation(robot_pose_at_image_capture_time + first_pose_position, visualization_plot)

        rotation_about_z = array([[0, 1, 0, 0],
                                  [-1, 0, 0, 0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])

        # Generate a transformed list of points for the largest contour
        transformed_contours, transformed_vectors, transformed_centroids = image_data.generate_transformed_contours(
            transformation=(robot_pose_at_image_capture_time @ rotation_about_z) + first_pose_position,
            imaging_depth=image_data.imaging_depth / 100)

        if len(transformed_contours) > 1:
            transformed_contours = [transformed_contours[0]]
        if len(transformed_centroids) > 1:
            transformed_centroids = [transformed_centroids[0]]

        if len(transformed_contours) == 0 or len(transformed_centroids) == 0:
            break

        visualization_plot.scatter3D(array(transformed_contours)[0, :, 0],
                                     array(transformed_contours)[0, :, 1],
                                     array(transformed_contours)[0, :, 2])

        visualization_plot.scatter3D(array(transformed_centroids)[0, :, 0],
                                     array(transformed_centroids)[0, :, 1],
                                     array(transformed_centroids)[0, :, 2],
                                     c='black', s=75
                                     )

        this_point_cloud.append([tuple(point) for point in transformed_contours[0][::ceil(
            len(registered_data.image_data.contours_in_image[0]) / 100)]])

        """this_point_cloud.append(
            [(point[0], point[1], placement_index) for point in registered_data.image_data.contours_in_image[0][::ceil(
                len(registered_data.image_data.contours_in_image[0]) / 150)]])
        placement_index = placement_index + 25"""

# Calculate the triangles in the point cloud
point_cloud_triangles = create_mesh_triangles(this_point_cloud, visualization_plot)

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
