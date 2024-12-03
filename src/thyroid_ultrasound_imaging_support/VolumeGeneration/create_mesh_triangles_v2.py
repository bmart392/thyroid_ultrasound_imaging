# Import standard packages
from numpy import array
from mpl_toolkits.mplot3d import Axes3D
from statistics import median
from copy import deepcopy

# Import custom packages
from thyroid_ultrasound_imaging_support.VolumeGeneration.calc_index_distance import calc_index_distance
from thyroid_ultrasound_imaging_support.VolumeGeneration.create_convex_triangles_from_3d_points import \
    create_convex_triangles_from_3d_points
from thyroid_ultrasound_imaging_support.VolumeGeneration.find_closest_point import find_closest_point
from thyroid_ultrasound_imaging_support.VolumeGeneration.plot_shapes import plot_shapes
from thyroid_ultrasound_imaging_support.VolumeGeneration.wrapping_range import wrapping_range
from thyroid_ultrasound_imaging_support.VolumeGeneration.CrossContourLine import CrossContourLine, VALUE_DATA, \
    INDEX_DATA, Point, THIS_CONTOUR, NEXT_CONTOUR
from thyroid_ultrasound_imaging_support.VolumeGeneration.SingleContourLine import SingleContourLine
from thyroid_ultrasound_imaging_support.VolumeGeneration.Triangle import Triangle

# Define color constants
POINT_COLOR: str = 'black'  # Point colors used to color the points on each contour
CONTOUR_LINE: str = 'black'  # Line color used to color the lines that exist on contours
END_FACE_LINE_COLOR: str = 'purple'
COMMON_CLOSEST_POINT_COLOR: str = 'green'
QUADRILATERAL_CLOSING_LINE_COLOR: str = 'orange'
GAP_FILLING_LINE_COLOR: str = 'blue'

PREVIOUS_COMMON: int = 0
NEXT_COMMON: int = 1


def create_mesh_triangles_v2(point_cloud: list, progress_plot: Axes3D = None, centroids=None):
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

    # Calculate the distance between each centroid as a vector for each contour in the point cloud
    contour_adjustment_vectors = [tuple([a - b for a, b in zip(centroids[i + 1], centroids[i])]) for i in
                                  range(len(centroids) - 1)]

    # Calculate the magnitude of each contour adjustment vector
    contour_adjustment_magnitudes = [(c[0] ** 2 + c[1] ** 2 + c[2] ** 2) ** 0.5 for c in contour_adjustment_vectors]

    # Calculate the median contour adjustment magnitude
    median_contour_adjustment_magnitude = median(contour_adjustment_magnitudes)

    # Define a list to store the triangles generated from the algorithm
    list_of_triangles = []

    # Define lists to store the found pairs of points
    list_of_forward_pairs = []
    list_of_backward_pairs = []
    list_of_common_pairs = []

    # Define a list of shapes that have been plotted
    shapes_that_have_been_plotted = []

    # Create the triangles for the face of the first slice
    list_of_triangles = list_of_triangles + create_triangles_on_slice_face(point_cloud[0], progress_plot=progress_plot,
                                                                           plot_points=True)

    # For each contour in point_cloud
    for i in range(len(point_cloud) - 1):

        # Calculate the contour adjustment factor
        contour_adjustment = (0, 0, 0)
        if abs((contour_adjustment_magnitudes[i] / median_contour_adjustment_magnitude) - 1) <= 0.25:
            contour_adjustment = (contour_adjustment_vectors[i][0], 0, contour_adjustment_vectors[i][2])

        # Create lists of the unpaired points on this and the next contour
        unpaired_points_on_this_contour: list = deepcopy(point_cloud[i])
        unpaired_points_on_next_contour: list = deepcopy(point_cloud[i + 1])

        # If plotting the progress, plot the next contour
        adjusted_point_cloud = [tuple([point[j] + contour_adjustment[j] for j in range(len(point))]) for point in point_cloud[i + 1]]
        plot_contour_points(point_cloud[i + 1], progress_plot, POINT_COLOR)

        # Find the first common pair of points
        previous_common_pair = find_common_pair_of_closest_points(unpaired_points_on_this_contour,
                                                                  unpaired_points_on_next_contour,
                                                                  point_cloud[i], point_cloud[i + 1],
                                                                  contour_adjustment)

        # Update the corresponding lists and plot it if necessary
        clean_up_line(previous_common_pair, unpaired_points_on_this_contour, unpaired_points_on_next_contour,
                      shapes_that_have_been_plotted, progress_plot)

        # While points have not been paired,
        while len(unpaired_points_on_this_contour) + len(unpaired_points_on_next_contour) > 0:

            # Find the next common pair of points
            new_common_pair = find_common_pair_of_closest_points(unpaired_points_on_this_contour,
                                                                 unpaired_points_on_next_contour,
                                                                 point_cloud[i], point_cloud[i + 1],
                                                                 contour_adjustment)

            # Plot the new line for clarity
            plot_single_line(new_common_pair, shapes_that_have_been_plotted, progress_plot)

            # Calculate the index distances between the common pairs
            index_distances = calc_index_distances_between_common_pairs(previous_common_pair, new_common_pair,
                                                                        point_cloud[i], point_cloud[i + 1])

            # If the common pairs are parallel and next to one another,
            if sum(index_distances) == 2:

                # Create new triangles from the two parallel lines
                new_triangles = create_triangles_from_quadrilateral(previous_common_pair, new_common_pair)

                # Add them to the list of triangles on the shape
                list_of_triangles = list_of_triangles + [t.convert_to_vertices() for t in new_triangles]

                # Update the corresponding lists and plot if necessary
                clean_up_generated_triangles(new_triangles, unpaired_points_on_this_contour,
                                             unpaired_points_on_next_contour, shapes_that_have_been_plotted,
                                             progress_plot)

            # If the common pairs are adjacent on one contour but separated by one or more points on the other,
            elif max(index_distances) > min(index_distances) == 1:

                # Define a variable for storing how many lines are connected to the previous and new common pair
                lines_connected_to_each_common = [[], []]

                # Determine which contour has the gap between the common pairs
                contour_with_gap = index_distances.index(max(index_distances))

                # Calculate the indices of the points between the two common pairs
                points_contained_in_the_gap = list(
                    range(previous_common_pair.get_data_of_point_given_contour(contour_with_gap, INDEX_DATA) + 1,
                          new_common_pair.get_data_of_point_given_contour(contour_with_gap, INDEX_DATA)))

                # Define a list of the points to search between
                possible_closest_point_objects = [previous_common_pair.get_point_from_contour(contour_with_gap + 1),
                                                  new_common_pair.get_point_from_contour(contour_with_gap + 1)]
                possible_closest_point_values = [p.value for p in possible_closest_point_objects]

                # For each index in the gap, find which of the two common pairs it is closest to
                for index in points_contained_in_the_gap:
                    # Create a point for the current index
                    point_on_contour_with_gap = Point(index, point_cloud[i + contour_with_gap][index])

                    # Find the closest point
                    _, closest_point_index = find_closest_point(point_on_contour_with_gap.value,
                                                                possible_closest_point_values, [],
                                                                contour_adjustment)

                    # Pull out the closest point as an object based on the index returned
                    closest_point_as_object = possible_closest_point_objects[closest_point_index]

                    # Create a new line between the closest point and the point on the separated contour
                    new_line = CrossContourLine(
                        line_color=GAP_FILLING_LINE_COLOR).add_data_based_on_contour_index(contour_with_gap,
                                                                                           point_on_contour_with_gap,
                                                                                           contour_with_gap + 1,
                                                                                           closest_point_as_object)

                    # Update the correct list with the new line
                    lines_connected_to_each_common[closest_point_index].append(new_line)

                    # Plot the new line if necessary
                    plot_single_line(new_line, shapes_that_have_been_plotted, progress_plot)

                # Define a list to store the triangles that will be generated
                new_triangles = []

                # For each set of lines found previously and its associated common pair, create triangles
                for list_of_lines, associated_common in zip(lines_connected_to_each_common,
                                                            (previous_common_pair, new_common_pair)):

                    # If there are lines connected to this common,
                    if len(list_of_lines) > 0:

                        # Append a new triangle that contains the associated common and the first line
                        new_triangles.append(
                            Triangle(associated_common, list_of_lines[0],
                                     SingleContourLine(associated_common.get_point_from_contour(contour_with_gap),
                                                       list_of_lines[0].get_point_from_contour(contour_with_gap),
                                                       contour_with_gap)))

                        # Make more triangles as long as more than one line is connected to the common
                        while len(list_of_lines) > 1:
                            # Append a new triangle that contains the first and second lines connected to the common
                            new_triangles.append(
                                Triangle(list_of_lines[0], list_of_lines[1],
                                         SingleContourLine(list_of_lines[0].get_point_from_contour(contour_with_gap),
                                                           list_of_lines[1].get_point_from_contour(contour_with_gap),
                                                           contour_with_gap)))

                            # Pop out the first line in the list to ensure that the looping ends
                            list_of_lines.pop(0)

                # Determine if the which lines should be used to create the final closing triangles
                if len(lines_connected_to_each_common[PREVIOUS_COMMON]) > 0:
                    previous_line_to_use = lines_connected_to_each_common[PREVIOUS_COMMON][0]
                else:
                    previous_line_to_use = previous_common_pair
                if len(lines_connected_to_each_common[NEXT_COMMON]) > 0:
                    next_line_to_use = lines_connected_to_each_common[NEXT_COMMON][0]
                else:
                    next_line_to_use = new_common_pair

                # Create triangles from the remaining two lines
                new_triangles = new_triangles + create_triangles_from_quadrilateral(previous_line_to_use, next_line_to_use)

                # Update the corresponding lists and plot if necessary
                clean_up_generated_triangles(new_triangles, unpaired_points_on_this_contour,
                                             unpaired_points_on_next_contour, shapes_that_have_been_plotted,
                                             progress_plot)

                # new_triangles.append(Triangle(previous_line_to_use, next_line_to_use,
                #                               SingleContourLine(
                #                                   previous_line_to_use.get_point_from_contour(contour_with_gap),
                #                                   next_line_to_use.get_point_from_contour(contour_with_gap),
                #                                   contour_with_gap)))

            # If the common pairs are separated by one or more points on both contours but on gap has less point,
            elif max(index_distances) > min(index_distances) > 1:

                # Determine which contour has the smallest gap
                contour_with_small_gap = index_distances.index(min(index_distances))
                contour_with_large_gap = index_distances.index(min(index_distances))

                # Find the closest point for each point on the smaller gap side

                # Between each of those lines and the border lines, repeat the process from above

                pass

            else:

                # Update the corresponding lists and plot it if necessary
                clean_up_line(new_common_pair, unpaired_points_on_this_contour, unpaired_points_on_next_contour,
                              shapes_that_have_been_plotted, progress_plot)

            # Set the previous common pair
            previous_common_pair = new_common_pair

        #     # Ensure that the iterators do not exceed the lengths of the lists
        #     this_iterator = this_iterator % len(unpaired_points_on_this_contour)
        #     next_iterator = next_iterator % len(unpaired_points_on_next_contour)
        #
        #     # Define a list to store the paired points found in the next point cloud
        #     list_of_paired_points = []
        #
        #     # Define a variable to store the index of the previously found paired point
        #     previous_paired_point_index = 0
        #
        #     # Define the list of indices to use to create the triangles
        #     current_point_cloud_indices = list(range(len(point_cloud[i]))) + [0]
        #
        #     # For each index in the current point cloud
        #     for j in range(len(current_point_cloud_indices)):
        #         # Get the current point index and the previous point index
        #         previous_point_index = current_point_cloud_indices[j - 1]
        #         current_point_index = current_point_cloud_indices[j]
        #
        #         # Find the closest point in the next contour
        #         paired_point, paired_point_index = find_closest_point(point_cloud[i][current_point_index],
        #                                                               point_cloud[i + 1],
        #                                                               list_of_paired_points,
        #                                                               centroid_adjustment=contour_adjustment)
        #
        #         # Add the pair to the appropriate list
        #         list_of_forward_pairs.append((current_point_index, paired_point_index))
        #
        #         # # Plot the forwards line
        #         # plot_shapes([(point_cloud[i][current_point_index], point_cloud[i+1][paired_point_index])],
        #         #             progress_plot, 'black')
        #
        #         # # If this is the first point in the contour, add the closest point to the list of paired points
        #         # if current_point_index == 0 and previous_point_index == 0:
        #         #     list_of_paired_points.append(paired_point)
        #         #
        #         # # Otherwise
        #         # else:
        #         #
        #         #     # If the paired point is one over from the previous paired point
        #         #     if abs(calc_index_distance(paired_point_index, previous_paired_point_index, point_cloud[i + 1])) == 1:
        #         #         # Form two triangles to close the quadrilateral
        #         #         new_triangles = [(point_cloud[i][previous_point_index],
        #         #                           point_cloud[i + 1][previous_paired_point_index],
        #         #                           paired_point),
        #         #                          (point_cloud[i][current_point_index],
        #         #                           point_cloud[i][previous_point_index],
        #         #                           paired_point)]
        #         #
        #         #         # Set the color to use when plotting the triangles
        #         #         plot_color = 'orange'
        #         #
        #         #         # Update the list of paired points
        #         #         list_of_paired_points.append(paired_point)
        #         #
        #         #     # Otherwise,
        #         #     else:
        #         #         # If the last loop is in progress AND
        #         #         # the closest point on this iteration is the same as the closest point on the last iteration,
        #         #         if j == len(current_point_cloud_indices) - 1 and previous_paired_point_index == paired_point_index:
        #         #
        #         #             # Set the temporary previous paired point index as the index of the current paired point index
        #         #             temp_previous_paired_point_index = paired_point_index
        #         #
        #         #             # Set the temporary next paired point index as the index of the first paired point index
        #         #             temp_next_paired_point_index = point_cloud[i + 1].index(list_of_paired_points[0])
        #         #
        #         #         # Otherwise,
        #         #         else:
        #         #
        #         #             # Set the temporary previous paired point index as the previous paired point index
        #         #             temp_previous_paired_point_index = previous_paired_point_index
        #         #
        #         #             # Set the temporary next paired point index as the current paired point index
        #         #             temp_next_paired_point_index = paired_point_index
        #         #
        #         #         # For each index between the temporary previous paired point index and the temporary next paired
        #         #         # point index (accounting for wrapping around the end of the list),
        #         #         for k in wrapping_range(temp_previous_paired_point_index, temp_next_paired_point_index,
        #         #                                 point_cloud[i + 1]):
        #         #
        #         #             # Create a new triangle
        #         #             new_triangle = (point_cloud[i][previous_point_index],
        #         #                             point_cloud[i + 1][k],
        #         #                             point_cloud[i + 1][k + 1])
        #         #
        #         #             # Update the list of triangles
        #         #             list_of_triangles.append(new_triangle)
        #         #
        #         #             # Plot the new triangle, if required
        #         #             if progress_plot is not None:
        #         #                 plot_shapes([new_triangle], progress_plot, 'green')
        #         #
        #         #             # Update the list of paired points when necessary
        #         #             if k != previous_paired_point_index:
        #         #                 list_of_paired_points.append(point_cloud[i + 1][k])
        #         #
        #         #         # Create a triangle between the
        #         #         new_triangles = [(point_cloud[i][current_point_index],
        #         #                           point_cloud[i][previous_point_index],
        #         #                           paired_point)]
        #         #         plot_color = 'purple'
        #         #
        #         #     # Add the new triangles to the list of triangles
        #         #     list_of_triangles = list_of_triangles + new_triangles
        #         #
        #         #     # Plot the new triangles if necessary
        #         #     if progress_plot is not None:
        #         #         plot_shapes(new_triangles, progress_plot, plot_color)
        #
        #         # Save the index of this paired point
        #         previous_paired_point_index = paired_point_index
        #
        #     # Define the list of indices in the next point cloud
        #     next_point_cloud_indices = list(range(len(point_cloud[i + 1]))) + [0]
        #
        #     # For each index in the next point cloud
        #     for k in range(len(next_point_cloud_indices)):
        #         current_point_index = next_point_cloud_indices[k]
        #
        #         paired_point_backwards, paired_point_index_backwards = find_closest_point(
        #             point_cloud[i + 1][current_point_index],
        #             point_cloud[i],
        #             list_of_paired_points,
        #             centroid_adjustment=tuple([-1 * x for x in contour_adjustment])
        #         )
        #
        #         # Add the paired point to the list
        #         list_of_backward_pairs.append((paired_point_index_backwards, current_point_index))
        #
        #         # # Plot the backwards line
        #         # plot_shapes([(point_cloud[i+1][paired_point_index], point_cloud[i][paired_point_index_backwards])],
        #         #             progress_plot, 'blue')
        #
        # for point_pair in list_of_forward_pairs:
        #     if point_pair in list_of_backward_pairs:
        #         list_of_common_pairs.append(point_pair)
        #         list_of_forward_pairs.pop(list_of_forward_pairs.index(point_pair))
        #         list_of_backward_pairs.pop(list_of_backward_pairs.index(point_pair))
        #
        # for list_of_points, color in zip([list_of_common_pairs, list_of_forward_pairs, list_of_backward_pairs],
        #                                  ['green', 'orange', 'blue']):
        #     for point_pair in list_of_points:
        #         plot_shapes([(point_cloud[i][point_pair[0]], point_cloud[i + 1][point_pair[1]])], progress_plot, color)

    # Create the triangles for the face of the last slice
    list_of_triangles = list_of_triangles + create_triangles_on_slice_face(point_cloud[-1], progress_plot=progress_plot)

    # Return the list of triangles
    return list_of_triangles


def plot_contour_points(contour_points: list, progress_plot: Axes3D = None, point_color: str = POINT_COLOR):
    # Plot the first slice if necessary
    if progress_plot is not None:
        array_contour = array(contour_points)
        progress_plot.scatter(array_contour[:, 0],
                              array_contour[:, 1],
                              array_contour[:, 2],
                              color=point_color)


def create_triangles_on_slice_face(contour_points: list, line_color: str = END_FACE_LINE_COLOR,
                                   progress_plot: Axes3D = None, plot_points: bool = False,
                                   point_color: str = POINT_COLOR) -> list:
    # Plot the first slice if necessary
    if plot_points:
        plot_contour_points(contour_points, progress_plot, point_color)

    # Create the triangles for the face of the first slice
    new_triangles = create_convex_triangles_from_3d_points(contour_points)

    # If the progress plot is given,
    if progress_plot is not None:
        # Plot the triangles
        plot_shapes(new_triangles, progress_plot, line_color)

    return new_triangles


def find_common_pair_of_closest_points(unpaired_points_on_this_contour, unpaired_points_on_next_contour,
                                       all_points_on_this_contour, all_points_on_next_contour,
                                       centroid_adjustment_vector):
    for point_index, point in enumerate(unpaired_points_on_this_contour):
        # Find the closest point in the next contour
        forward_paired_point, forward_paired_point_index = find_closest_point(
            point, unpaired_points_on_next_contour, [], centroid_adjustment=centroid_adjustment_vector)

        # Find the closest point in this contour of the point in the next contour
        backward_paired_point, backward_paired_point_index = find_closest_point(
            forward_paired_point, unpaired_points_on_this_contour, [],
            centroid_adjustment=tuple([-1 * x for x in centroid_adjustment_vector])
        )
        if point_index == backward_paired_point_index:
            return CrossContourLine(
                Point(all_points_on_this_contour.index(backward_paired_point), backward_paired_point),
                Point(all_points_on_next_contour.index(forward_paired_point), forward_paired_point),
                COMMON_CLOSEST_POINT_COLOR)

    return None


def calc_index_distances_between_common_pairs(pair_1: CrossContourLine, pair_2: CrossContourLine,
                                              this_contour: list, next_contour: list) -> tuple:
    return calc_index_distance(pair_2.point_1.index,
                               pair_1.point_1.index, this_contour), \
           calc_index_distance(pair_2.point_2.index,
                               pair_1.point_2.index, next_contour)


def create_triangles_from_quadrilateral(line_1: CrossContourLine, line_2: CrossContourLine) -> list:
    diagonal_line = CrossContourLine(line_1.point_2, line_2.point_1, QUADRILATERAL_CLOSING_LINE_COLOR)
    return [Triangle(cross_contour_line_a=line_1,
                     cross_contour_line_b=diagonal_line,
                     single_contour_line=SingleContourLine(line_2.point_1, line_1.point_1, THIS_CONTOUR, CONTOUR_LINE)),
            Triangle(cross_contour_line_a=line_2,
                     single_contour_line=SingleContourLine(line_2.point_2, line_1.point_2, NEXT_CONTOUR, CONTOUR_LINE),
                     cross_contour_line_b=diagonal_line)]


def inverse_contour_selection(contour: int):
    return (contour + 1) % 2


def clean_up_line(line: CrossContourLine, unpaired_points_on_this_contour: list,
                  unpaired_points_on_next_contour: list, lines_that_have_been_plotted: list,
                  progress_plot: Axes3D):
    # Pop out the common points
    try:
        unpaired_points_on_this_contour.pop(unpaired_points_on_this_contour.index(line.point_1.value))
    except ValueError:
        pass
    try:
        unpaired_points_on_next_contour.pop(unpaired_points_on_next_contour.index(line.point_2.value))
    except ValueError:
        pass

    # If the line has not already been plotted and update the list of lines that have been plotted
    plot_single_line(line, lines_that_have_been_plotted, progress_plot)


def clean_up_generated_triangles(list_of_triangles: list, unpaired_points_on_this_contour: list,
                                 unpaired_points_on_next_contour: list, lines_that_have_been_plotted: list,
                                 progress_plot: Axes3D):
    for triangle in list_of_triangles:
        for line in triangle.get_constituent_shapes():
            clean_up_line(line, unpaired_points_on_this_contour, unpaired_points_on_next_contour,
                          lines_that_have_been_plotted, progress_plot)


def plot_single_line(line: CrossContourLine, lines_that_have_been_plotted: list, progress_plot: Axes3D):
    # If the line has not already been plotted,
    if line not in lines_that_have_been_plotted:

        # Add the new common pair to the list of shapes that has already been plotted
        lines_that_have_been_plotted.append(line)

        # Plot the line
        if progress_plot is not None:
            plot_shapes(line.to_shape(), progress_plot,
                        line.color)

# # If the common pairs are separated by one point on either contour
# if sum(index_distances) == 3:
#     # Determine which contour has the separation
#     contour_with_separation = index_distances.index(max(index_distances))
#
#     points_contained_in_separation = list(
#         range(previous_common_pair.get_data_of_point_given_contour(contour_with_separation, INDEX_DATA) + 1,
#               new_common_pair.get_data_of_point_given_contour(contour_with_separation, INDEX_DATA)))
#
#     new_lines = [(previous_common_pair.get_data_of_point_given_contour(
#         contour_with_separation + 1, VALUE_DATA),
#                   point_cloud[i + contour_with_separation][
#                       previous_common_pair.get_data_of_point_given_contour(contour_with_separation,
#                                                                            INDEX_DATA) + 1]),
#         (new_common_pair.get_data_of_point_given_contour(
#             contour_with_separation + 1, VALUE_DATA),
#          point_cloud[i + contour_with_separation][
#              previous_common_pair.get_data_of_point_given_contour(contour_with_separation,
#                                                                   INDEX_DATA) + 1])]
#
#     plot_shapes(new_lines, progress_plot, QUADRILATERAL_CLOSING_LINE_COLOR)
#
#     continue
