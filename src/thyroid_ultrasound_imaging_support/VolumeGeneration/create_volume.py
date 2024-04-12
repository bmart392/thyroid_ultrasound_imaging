from numpy import array, zeros
from numpy.linalg import norm

from stl import mesh

from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D

from thyroid_ultrasound_imaging_support.Boundaries.create_convex_triangles_from_points import \
    create_convex_triangles_from_points
from thyroid_ultrasound_imaging_support.RegisteredData.RegisteredData import RegisteredData, ImageData
from thyroid_ultrasound_imaging_support.RegisteredData.load_folder_of_saved_registered_data import \
    load_folder_of_saved_registered_data


def create_volume(point_cloud: list, vectors: list, progress_plot):
    """

    Parameters
    ----------
    point_cloud :
        A multi-dimensional list where the points from each image are in their own list
    vectors :
        A multi-dimensional list where the vectors from each image are in their own list and correspond to the points
        in the point_cloud.

    Returns
    -------

    """

    # Define a list to store the triangles generated from the algorithm
    list_of_triangles = []

    # Define a list to store the vectors for each triangle
    list_of_normal_vectors = []

    # Define the dot colors to use to plot
    point_colors = ['red', 'blue']

    array_contour = array(point_cloud[0])
    progress_plot.scatter(array_contour[:, 0],
                          array_contour[:, 1],
                          array_contour[:, 2],
                          color=point_colors[0 % len(point_colors)])

    # For each contour in point_cloud
    for i in range(len(point_cloud) - 1):

        array_contour = array(point_cloud[i + 1])
        progress_plot.scatter(array_contour[:, 0],
                              array_contour[:, 1],
                              array_contour[:, 2],
                              color=point_colors[(i + 1) % len(point_colors)])

        # Define a list to store the paired points found in the next point cloud
        list_of_paired_points = []

        # Define a variable to store the index of the previously found paired point
        previous_paired_point_index = 0

        # Define the list of indices to use to create the triangles
        current_point_cloud_indices = list(range(len(point_cloud[i]))) + [0]

        # For each point in the contour, find the closest point and create triangles
        for j in range(len(current_point_cloud_indices)):

            previous_point_index = current_point_cloud_indices[j - 1]
            current_point_index = current_point_cloud_indices[j]

            # Find the closest point in the next contour
            paired_point, paired_point_index = find_closest_point(point_cloud[i][current_point_index],
                                                                  point_cloud[i + 1],
                                                                  list_of_paired_points)

            # If this is the first point in the contour
            if current_point_index == 0 and previous_point_index == 0:
                # Add the closest point to the list of paired points
                list_of_paired_points.append(paired_point)

            # Otherwise
            else:

                # If the paired point is one over from the previous paired point
                if abs(calc_index_distance(paired_point_index, previous_paired_point_index, point_cloud[i + 1])) == 1:
                    # Form two triangles to close the quadrilateral
                    list_of_triangles, result_triangles = \
                        create_triangles_from_quadrilateral(point_cloud[i][previous_point_index],
                                                            point_cloud[i][current_point_index],
                                                            point_cloud[i + 1][previous_paired_point_index],
                                                            paired_point,
                                                            list_of_triangles)
                    plot_triangles(result_triangles, progress_plot, 'orange')

                    list_of_paired_points.append(paired_point)

                # If the paired point is the same as the previous point
                else:  # paired_point == list_of_paired_points[-1]:
                    if j == len(current_point_cloud_indices) - 1 and previous_paired_point_index == paired_point_index:
                        temp_previous_paired_point_index = paired_point_index
                        temp_next_paired_point_index = point_cloud[i + 1].index(list_of_paired_points[0])
                    else:
                        temp_previous_paired_point_index = previous_paired_point_index
                        temp_next_paired_point_index = paired_point_index

                    for k in wrapping_range(temp_previous_paired_point_index, temp_next_paired_point_index, point_cloud[i + 1]):
                        new_triangle = (point_cloud[i][previous_point_index],
                                        point_cloud[i + 1][k],
                                        point_cloud[i + 1][k + 1])
                        list_of_triangles.append(new_triangle)
                        plot_triangles([new_triangle], progress_plot, 'green')

                        if k != previous_paired_point_index:
                            list_of_paired_points.append(point_cloud[i + 1][k])

                    # Create a triangle between the
                    new_triangle = (point_cloud[i][previous_point_index],
                                    point_cloud[i][current_point_index],
                                    paired_point)
                    list_of_triangles.append(new_triangle)
                    plot_triangles([new_triangle], progress_plot, 'purple')

            # Save the index of this paired point
            previous_paired_point_index = paired_point_index

    unique_triangles = []
    for triangle in list_of_triangles:
        if not any([(triangle[a], triangle[b], triangle[c]) in unique_triangles for a, b, c in zip([0, 0, 1, 1, 2, 2],
                                                                                               [1, 2, 1, 2, 0, 1],
                                                                                               [2, 1, 2, 0, 1, 0])]):
            unique_triangles.append(triangle)

    return list_of_triangles


def create_triangles_from_quadrilateral(previous_point_on_this_point_cloud: tuple,
                                        current_point_on_this_point_cloud: tuple,
                                        previous_paired_point: tuple,
                                        current_paired_point: tuple,
                                        destination_list: list):
    temp_result = [(previous_point_on_this_point_cloud, previous_paired_point, current_paired_point),
                   (current_point_on_this_point_cloud, previous_point_on_this_point_cloud, current_paired_point)]

    return destination_list + temp_result, temp_result


MODE_MIN: str = 'min'
MODE_MAX: str = 'max'

PREFER_POSITIVE: int = 1
PREFER_NEGATIVE: int = -1


def calc_index_distance(current_index: int, previous_index: int, source_list: list,
                        mode: str = MODE_MIN, direction_preference: int = PREFER_POSITIVE):
    # Calculate all the possible distances
    signed_distances = [current_index - previous_index,
                        current_index - (previous_index - len(source_list)),
                        current_index - (previous_index + len(source_list))]
    unsigned_distances = [abs(x) for x in signed_distances]

    # Find the distance using the requested method
    if mode == MODE_MIN:
        temp_distance = min(unsigned_distances)
    elif mode == MODE_MAX:
        temp_distance = max(unsigned_distances)
    else:
        raise ValueError('The given mode, ' + mode + ', is not a recognized mode.')

    # Ensure that the direction preference given is valid
    if not any([direction_preference == x for x in [PREFER_POSITIVE, PREFER_NEGATIVE]]):
        raise ValueError('The given direction preference, ' + str(direction_preference) +
                         ', is not a recognized preference.')

    # Return the appropriate value, accounting for preference when inverse values exist
    if direction_preference * temp_distance in signed_distances:
        return direction_preference * temp_distance
    else:
        return -direction_preference * temp_distance


def find_closest_point(reference_point: tuple, contour: list, previously_found_points: list):


    reference_point_array = array(reference_point)

    if len(previously_found_points) > 0:
        closest_point = previously_found_points[-1]
    else:
        closest_point = contour[0]
    closest_distance = calc_norm(closest_point, reference_point_array)

    for point in contour[1:]:

        if len(previously_found_points) == 0 or calc_index_distance(contour.index(point),
                                                                    contour.index(previously_found_points[-1]),
                                                                    contour) > 0:

            new_distance = calc_norm(point, reference_point_array)

            if new_distance < closest_distance:
                closest_point = point
                closest_distance = new_distance

            elif new_distance == closest_distance and (
                    point not in previously_found_points or len(previously_found_points) == len(contour)):
                closest_point = point
                closest_distance = new_distance

    return closest_point, contour.index(closest_point)

def calc_norm(pt_1: tuple, pt_2: array):
    return norm(array(pt_1) - pt_2)


def wrapping_range(index_1, index_2, source_list):
    if index_2 < index_1:
        return range(index_1 - len(source_list), index_2)
    else:
        return range(index_1, index_2)


def plot_triangles(triangles_to_plot: list, progress_plot, line_color):
    lines_to_plot = []

    for triangle in triangles_to_plot:
        indices = [0, 1, 2, 0]
        for i in range(3):
            vertex_a = triangle[indices[i]]
            vertex_b = triangle[indices[i + 1]]
            if not (vertex_a, vertex_b) in lines_to_plot or not (vertex_b, vertex_a) in lines_to_plot:
                lines_to_plot.append((vertex_a, vertex_b))

    for line in lines_to_plot:
        line_array = array(line)
        progress_plot.plot(line_array[:, 0],
                           line_array[:, 1],
                           line_array[:, 2],
                           color=line_color)
        plt.draw()
        a = 1


if __name__ == '__main__':

    # Read in the data
    list_of_registered_data = load_folder_of_saved_registered_data('/home/ben/thyroid_ultrasound_data/'
                                                                   'testing_and_validation/VolumeGenerationExperiment/'
                                                                   'VolumeData/'
                                                                   'VolumeData_2024-04-10_15-23-55-331210_2024-04-11_08-14-19-675417')

    # Create a variable to store the point cloud data
    point_cloud = []

    # Create a variable to define the x location of each point cloud
    placement_index = 0

    # For each registered data, pull out the contour and save it to the point cloud
    for registered_data in list_of_registered_data:
        registered_data: RegisteredData
        point_cloud.append(
            [(point[0], point[1], placement_index) for point in registered_data.image_data.contours_in_image[0][::round(len(registered_data.image_data.contours_in_image[0])/50)]])
        placement_index = placement_index + 25

    # Set the plot to interactive mode
    # plt.ion()

    # Create a new plot
    fig = plt.figure()
    visualization_plot = fig.add_subplot(projection='3d')
    visualization_plot.set_xlabel('X Label')
    visualization_plot.set_ylabel('Y Label')
    visualization_plot.set_zlabel('Z Label')

    # Calculate the triangles in the point cloud
    point_cloud_triangles = create_volume(point_cloud, [], visualization_plot)

    # Convert the triangles to an array
    point_cloud_triangles_as_arrays = array(point_cloud_triangles)

    # Create a mesh and add the triangles
    volume_mesh = mesh.Mesh(zeros(point_cloud_triangles_as_arrays.shape[0], dtype=mesh.Mesh.dtype))
    volume_mesh.vectors = point_cloud_triangles_as_arrays

    # Turn off interactive plotting
    plt.ioff()

    # Create a new plot
    figure = plt.figure()
    axes = mplot3d.Axes3D(figure)

    # Load the STL files and add the vectors to the plot
    collection = mplot3d.art3d.Poly3DCollection(volume_mesh.vectors)
    collection.set(facecolor=[1, 1, 1], alpha=1., linewidth=1, edgecolor=[0, 0, 0])
    axes.add_collection3d(collection)

    # Auto-scale to the mesh size
    scale = volume_mesh.points.flatten()
    axes.auto_scale_xyz(scale, scale, scale)

    # Show the plot to the screen
    plt.show()
