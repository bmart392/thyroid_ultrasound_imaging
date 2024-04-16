from math import ceil

from numpy import array, zeros, ndarray, cross
from numpy.linalg import norm
from pymeshfix import clean_from_file, clean_from_arrays

from stl import mesh

from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D

from thyroid_ultrasound_imaging_support.Boundaries.create_convex_triangles_from_points import \
    create_convex_triangles_from_points
from thyroid_ultrasound_imaging_support.RegisteredData.RegisteredData import RegisteredData, ImageData
from thyroid_ultrasound_imaging_support.RegisteredData.load_folder_of_saved_registered_data import \
    load_folder_of_saved_registered_data

# Import standard packages
from panda3d.core import Triangulator3


def create_mesh_triangles(point_cloud: list, progress_plot: Axes3D = None):
    """
    Calculates a closed
    Parameters
    ----------
    point_cloud :
        A multi-dimensional list where the points from each image are in their own list
    progress_plot :
        A 3D axes object to plot the progress of the function.

    Returns
    -------

    """

    # Define a list to store the triangles generated from the algorithm
    list_of_triangles = []

    # Define the dot colors to use to plot
    point_colors = ['red', 'blue']

    # Plot the first slice if necessary
    if progress_plot is not None:
        array_contour = array(point_cloud[0])
        progress_plot.scatter(array_contour[:, 0],
                              array_contour[:, 1],
                              array_contour[:, 2],
                              color=point_colors[0 % len(point_colors)])

    # Create the triangles for the face of the first slice
    new_triangles = create_convex_triangles_from_3d_points(point_cloud[0])
    list_of_triangles = list_of_triangles + new_triangles
    if progress_plot is not None:
        plot_triangles(new_triangles, progress_plot, point_colors[0])

    # For each contour in point_cloud
    for i in range(len(point_cloud) - 1):

        # If plotting the progress
        if progress_plot is not None:
            # Create an array version of the contour and plot each point on the contour
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

        # For each index in the current point cloud
        for j in range(len(current_point_cloud_indices)):

            # Get the current point index and the previous point index
            previous_point_index = current_point_cloud_indices[j - 1]
            current_point_index = current_point_cloud_indices[j]

            # Find the closest point in the next contour
            paired_point, paired_point_index = find_closest_point(point_cloud[i][current_point_index],
                                                                  point_cloud[i + 1],
                                                                  list_of_paired_points)

            # If this is the first point in the contour, add the closest point to the list of paired points
            if current_point_index == 0 and previous_point_index == 0:
                list_of_paired_points.append(paired_point)

            # Otherwise
            else:

                # If the paired point is one over from the previous paired point
                if abs(calc_index_distance(paired_point_index, previous_paired_point_index, point_cloud[i + 1])) == 1:
                    # Form two triangles to close the quadrilateral
                    new_triangles = [(point_cloud[i][previous_point_index],
                                      point_cloud[i + 1][previous_paired_point_index],
                                      paired_point),
                                     (point_cloud[i][current_point_index],
                                      point_cloud[i][previous_point_index],
                                      paired_point)]

                    # Set the color to use when plotting the triangles
                    plot_color = 'orange'

                    # Update the list of paired points
                    list_of_paired_points.append(paired_point)

                # Otherwise,
                else:
                    # If the last loop is in progress AND
                    # the closest point on this iteration is the same as the closest point on the last iteration,
                    if j == len(current_point_cloud_indices) - 1 and previous_paired_point_index == paired_point_index:

                        # Set the temporary previous paired point index as the index of the current paired point index
                        temp_previous_paired_point_index = paired_point_index

                        # Set the temporary next paired point index as the index of the first paired point index
                        temp_next_paired_point_index = point_cloud[i + 1].index(list_of_paired_points[0])

                    # Otherwise,
                    else:

                        # Set the temporary previous paired point index as the previous paired point index
                        temp_previous_paired_point_index = previous_paired_point_index

                        # Set the temporary next paired point index as the current paired point index
                        temp_next_paired_point_index = paired_point_index

                    # For each index between the temporary previous paired point index and the temporary next paired
                    # point index (accounting for wrapping around the end of the list),
                    for k in wrapping_range(temp_previous_paired_point_index, temp_next_paired_point_index,
                                            point_cloud[i + 1]):

                        # Create a new triangle
                        new_triangle = (point_cloud[i][previous_point_index],
                                        point_cloud[i + 1][k],
                                        point_cloud[i + 1][k + 1])

                        # Update the list of triangles
                        list_of_triangles.append(new_triangle)

                        # Plot the new triangle, if required
                        if progress_plot is not None:
                            plot_triangles([new_triangle], progress_plot, 'green')

                        # Update the list of paired points when necessary
                        if k != previous_paired_point_index:
                            list_of_paired_points.append(point_cloud[i + 1][k])

                    # Create a triangle between the
                    new_triangles = [(point_cloud[i][current_point_index],
                                      point_cloud[i][previous_point_index],
                                      paired_point)]
                    plot_color = 'purple'

                # Add the new triangles to the list of triangles
                list_of_triangles = list_of_triangles + new_triangles

                # Plot the new triangles if necessary
                if progress_plot is not None:
                    plot_triangles(new_triangles, progress_plot, plot_color)

            # Save the index of this paired point
            previous_paired_point_index = paired_point_index

    # Create the triangles for the face of the last slice
    new_triangles = create_convex_triangles_from_3d_points(point_cloud[i + 1])
    list_of_triangles = list_of_triangles + new_triangles
    if progress_plot is not None:
        plot_triangles(new_triangles, progress_plot, point_colors[(i + 1) % len(point_colors)])

    # Return the list of triangles
    return list_of_triangles


def calc_index_distance(index_a: int, index_b: int, source_list: list) -> float:
    """
    Calculates the distance between two indices and returns the minimum distance.
    Minimum is calculated with respect to the amount of distance that would
    need to be travelled to reach index A from index B, not by sign.

    A positive distance is returned when index A is greater than index B.

    Parameters
    ----------
    index_a :
        The reference index from which to calculate the distance.
    index_b :
        The index whose distance is not known.
    source_list :
        The list where the indices originate.
    Returns
    -------
    float :
        The distance between index A and B.
    """

    # Calculate the distance between the two indices
    basic_distance = index_a - index_b

    # Calculate the sign of the first distance
    try:
        sign_of_basic_distance = basic_distance / abs(basic_distance)
    except ZeroDivisionError:
        # If the first distance was zero, the sign is positive
        sign_of_basic_distance = 1

    # Add the first distance and the inverse distance to the list
    signed_distances = [basic_distance, int(basic_distance + (-sign_of_basic_distance * len(source_list)))]

    # Calculate the minimum distance by magnitude
    min_distance = min([abs(x) for x in signed_distances])

    # Return the proper distance
    if min_distance in signed_distances:
        return min_distance
    else:
        return -min_distance


def find_closest_point(reference_point: tuple, contour: list, previously_found_points: list) -> (tuple, int):
    """
    Finds the closest point to a reference point within a given contour of points.

    Parameters
    ----------
    reference_point :
        The point to use to find the closest point.
    contour :
        The list of points to search within where each point is formatted as a tuple.
    previously_found_points :
        A list containing points that have already been paired within the contour.

    Returns
    -------
    tuple :
        The point which was closest, as a tuple, and the index of the closest point within the contour, as an int.

    """

    # Declare a variable to store the reference point as an array for math later
    reference_point_array = array(reference_point)

    # If a list of previously found points was given
    if len(previously_found_points) > 0:

        # Set the closest point as the last point found
        closest_point = previously_found_points[-1]

        # Set the point at which to start looking through the contour
        search_start_point = 0

        # Set the flag to not skip checking the index distance between points
        skip_calc_index_distance = False

    else:
        # Set the closest point as the first point in the contour
        closest_point = contour[0]

        # Start searching at the point afterward
        search_start_point = 1

        # Set the flag to skip calculating the index distance
        skip_calc_index_distance = True

    # Calculate the distance of the closest point and the reference point
    closest_distance = calc_norm(closest_point, reference_point_array)

    # For each point in the contour
    for point in contour[search_start_point:]:

        # If not skipping this calculation step,
        if not skip_calc_index_distance:

            # If the index distance between the current point and the previously found point is negative,
            # skip this point
            if calc_index_distance(contour.index(point), contour.index(previously_found_points[-1]), contour) < 0:
                continue

        # Calculate the distance between the current point and the reference point
        new_distance = calc_norm(point, reference_point_array)

        # If the new distance is less than the closest distance
        if new_distance < closest_distance:
            # Save the new point and distance
            closest_point = point
            closest_distance = new_distance

    return closest_point, contour.index(closest_point)


def calc_norm(pt_1: tuple, pt_2: ndarray) -> ndarray:
    """
    Calculates the normal distance between two points.
    """
    return norm(array(pt_1) - pt_2)


def wrapping_range(index_1: int, index_2: int, source_list: list) -> range:
    """
    Creates a range of numbers for iterating through a list that will properly wrap around the end of the list.

    Parameters
    ----------
    index_1 :
        The index to start the range at.
    index_2 :
        The index to end the range at.
    source_list :
        The list that will be indexed through.

    Returns
    -------
    range :
        A range of indices.
    """

    # If the ending index is less than the starting index,
    if index_2 < index_1:
        # Return a range that uses negative indexing to wrap around
        return range(index_1 - len(source_list), index_2)
    else:
        # Otherwise return a standard range
        return range(index_1, index_2)


def create_convex_triangles_from_3d_points(list_of_points: list) -> list:
    """
    Creates a set of convex, non-overlapping triangles, from a set of points forming a polygon.
    Each triangle is returned as a tuple of three (x, y, z) coordinates.

    Parameters
    ----------
    list_of_points
        a list of (x, y, z) coordinates defining a closed polygon.
    """
    # Create a triangulator object
    triangulator = Triangulator3()

    # Iterate through each point in the list
    for point in list_of_points:
        # Add each point to the pool of vertices and then to the current polygon
        triangulator.addPolygonVertex(triangulator.addVertex(x=point[0], y=point[1], z=point[2]))

    # Create the triangles of the polygons
    triangulator.triangulate()

    # Define list to store polygons
    list_of_triangles = []

    # Add the indices for each triangle to the list of triangle indices
    for i in range(triangulator.getNumTriangles()):
        list_of_triangles.append([triangulator.getTriangleV0(i),
                                  triangulator.getTriangleV1(i),
                                  triangulator.getTriangleV2(i)])
        for j in range(len(list_of_triangles[-1])):
            vertex = triangulator.getVertex(list_of_triangles[-1][j])
            list_of_triangles[-1][j] = (vertex.x, vertex.y, vertex.z)
        list_of_triangles[-1] = tuple(list_of_triangles[-1])

    # Replace the triangle indices with the points themselves
    return list_of_triangles


def plot_triangles(triangles_to_plot: list, progress_plot: Axes3D, line_color: str) -> None:
    """
    Interactively plots triangles on a 3D graph as 3 individual lines.

    Parameters
    ----------
    triangles_to_plot :
        A list of triangles to add to the plot given as a list of tuples. A single triangle is given as a tuple of the
        three vertices. Each vertex of the triangle is given as a tuple.
    progress_plot :
        The axes object on which to plot the triangles.
    line_color :
        The line color to use to plot the triangles.
    """

    # Define a list of lines to plot
    lines_to_plot = []

    # For each triangle in the list of triangles to plot
    for triangle in triangles_to_plot:

        # Create a list of indices to iterate through
        indices = [0, 1, 2, 0]

        # For each vertex in the triangle
        for i in range(3):

            # Define variables to store the two vertexes making up each side of the triangle
            vertex_a = triangle[indices[i]]
            vertex_b = triangle[indices[i + 1]]

            # If the line between these two vertices is not already in the list of lines to plot,
            # Add it to the list of lines to plot
            if not (vertex_a, vertex_b) in lines_to_plot or not (vertex_b, vertex_a) in lines_to_plot:
                lines_to_plot.append((vertex_a, vertex_b))

    # For each line to plot
    for line in lines_to_plot:
        # Create an array from the line
        line_array = array(line)

        # Plot the line
        progress_plot.plot(line_array[:, 0],
                           line_array[:, 1],
                           line_array[:, 2],
                           color=line_color)

        # Update the plot
        plt.draw()


if __name__ == '__main__':

    # Read in the data
    list_of_registered_data = load_folder_of_saved_registered_data('/home/ben/thyroid_ultrasound_data/'
                                                                   'testing_and_validation/VolumeGenerationExperiment/'
                                                                   'VolumeData/'
                                                                   'VolumeData_2024-04-10_15-23-55-331210_2024-04-11_08-14-19-675417')

    # Create a variable to store the point cloud data
    this_point_cloud = []

    # Create a variable to define the x location of each point cloud
    placement_index = 0

    # For each registered data, pull out the contour and save it to the point cloud
    for registered_data in list_of_registered_data:
        registered_data: RegisteredData

        # Pull out the corresponding data
        image_data = registered_data.image_data
        robot_pose_at_image_capture_time = registered_data.robot_pose.robot_pose

        # Generate a transformed list of points for the largest contour
        transformed_contours, transformed_vectors = image_data.generate_transformed_contours(
            transformation=robot_pose_at_image_capture_time,
            imaging_depth=image_data.imaging_depth / 100)

        this_point_cloud.append([tuple(point) for point in transformed_contours[0][::ceil(
                len(registered_data.image_data.contours_in_image[0]) / 100)]])

        """this_point_cloud.append(
            [(point[0], point[1], placement_index) for point in registered_data.image_data.contours_in_image[0][::ceil(
                len(registered_data.image_data.contours_in_image[0]) / 150)]])
        placement_index = placement_index + 25"""

    # Set the plot to interactive mode
    # plt.ion()

    # Create a new plot
    """fig = plt.figure()
    visualization_plot = fig.add_subplot(projection='3d')
    visualization_plot.set_xlabel('X Label')
    visualization_plot.set_ylabel('Y Label')
    visualization_plot.set_zlabel('Z Label')"""

    # Calculate the triangles in the point cloud
    point_cloud_triangles = create_mesh_triangles(this_point_cloud, None)  # visualization_plot)

    print("Number of faces: " + str(len(point_cloud_triangles)))

    # Convert the triangles to an array
    point_cloud_triangles_as_arrays = array(point_cloud_triangles)

    # Create a mesh and add the triangles
    volume_mesh = mesh.Mesh(zeros(point_cloud_triangles_as_arrays.shape[0], dtype=mesh.Mesh.dtype))
    volume_mesh.vectors = point_cloud_triangles_as_arrays

    volume_mesh.save('/home/ben/Desktop/test3.stl')

    unrepaired_mesh = mesh.Mesh.from_file('/home/ben/Desktop/test3.stl')
    clean_from_file('/home/ben/Desktop/test3.stl', '/home/ben/Desktop/test3_repaired.stl')
    repaired_mesh = mesh.Mesh.from_file('/home/ben/Desktop/test3_repaired.stl')

    volume, cog, inertia = repaired_mesh.get_mass_properties()
    print("Volume (mm^3)                           = {0}".format(volume / 10**-9))
    print("Volume (mL)                             = {0}".format(volume * 10**6))
    print("Position of the center of gravity (COG) = {0}".format(cog))
    print("Inertia matrix at expressed at the COG  = {0}".format(inertia[0, :]))
    print("                                          {0}".format(inertia[1, :]))
    print("                                          {0}".format(inertia[2, :]))

    """# Turn off interactive plotting
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
    plt.show()"""
