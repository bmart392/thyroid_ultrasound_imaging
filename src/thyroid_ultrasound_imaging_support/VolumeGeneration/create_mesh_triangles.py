# Import standard packages
from numpy import array
from mpl_toolkits.mplot3d import Axes3D
from statistics import median

# Import custom packages
from thyroid_ultrasound_imaging_support.VolumeGeneration.calc_index_distance import calc_index_distance
from thyroid_ultrasound_imaging_support.VolumeGeneration.create_convex_triangles_from_3d_points import \
    create_convex_triangles_from_3d_points
from thyroid_ultrasound_imaging_support.VolumeGeneration.find_closest_point import find_closest_point
from thyroid_ultrasound_imaging_support.VolumeGeneration.plot_triangles import plot_triangles
from thyroid_ultrasound_imaging_support.VolumeGeneration.wrapping_range import wrapping_range


def create_mesh_triangles(point_cloud: list, progress_plot: Axes3D = None, centroids = None):
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

    contour_adjustments = [tuple([a - b for a, b in zip(centroids[i + 1], centroids[i])]) for i in range(len(centroids) - 1)]
    contour_adjustments_magnitude = [(c[0]**2 + c[1]**2 + c[2]**2)**0.5 for c in contour_adjustments]
    median_contour_adjustment = median(contour_adjustments_magnitude)

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

            if abs(contour_adjustments_magnitude[i] - median_contour_adjustment) > abs(median_contour_adjustment) * 0.25:
                this_adjustment = (0, 0, 0)
            else:
                this_adjustment = contour_adjustments[i]

            # Find the closest point in the next contour
            paired_point, paired_point_index = find_closest_point(point_cloud[i][current_point_index],
                                                                  point_cloud[i + 1],
                                                                  list_of_paired_points,
                                                                  centroid_adjustment=this_adjustment)

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
