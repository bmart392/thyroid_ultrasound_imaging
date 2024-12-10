# Import standard packages
from numpy import array, sign
from mpl_toolkits.mplot3d import Axes3D
from statistics import median
from copy import copy, deepcopy

# Import custom packages
from thyroid_ultrasound_imaging_support.VolumeGeneration.calc_index_distance import calc_index_distance
from thyroid_ultrasound_imaging_support.VolumeGeneration.create_convex_triangles_from_3d_points import \
    create_convex_triangles_from_3d_points
from thyroid_ultrasound_imaging_support.VolumeGeneration.find_closest_point import find_closest_point
from thyroid_ultrasound_imaging_support.VolumeGeneration.plot_shapes import plot_shapes
from thyroid_ultrasound_imaging_support.VolumeGeneration.wrapping_range import wrapping_range
from thyroid_ultrasound_imaging_support.VolumeGeneration.CrossContourLine import CrossContourLine, \
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

# Define constants for indexing stored data
PREVIOUS_COMMON: int = 0
NEXT_COMMON: int = 1
OBJECT: int = 0
NUM_TIMES_USED: int = 1

# Define constants for each scenario of the two common lines
LINES_FORM_QUADRILATERAL: str = 'quadrilateral'
NO_GAP_AND_GAP: str = 'no gap and gap'
SINGLE_POINT_GAP_AND_GAP: str = 'single point gap and gap'

# Define constants for how connected a common pair is
UNCONNECTED: int = 0
CONNECTED_POSITIVE: int = 4
CONNECTED_NEGATIVE: int = -3
FULLY_CONNECTED: int = CONNECTED_POSITIVE + CONNECTED_NEGATIVE

# TODO - HIGH - Implement some form of external point prioritization


def create_mesh_triangles_v2(point_cloud: list, progress_plot: Axes3D = None, centroids=None,
                             external_points=None):
    """
    Calculates a closed

    Parameters
    ----------
    point_cloud :
        A multi-dimensional list where the points from each image are in their own list
    progress_plot :
        A 3D axes object to plot the progress of the function.
    centroids :
        A list of each centroid for each contour.
    external_points :
        A list of indices of the external points on each contour.

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

    # Define a list of shapes that have been plotted
    shapes_that_have_been_plotted = []

    # Create the triangles for the face of the first slice
    list_of_triangles = list_of_triangles + create_triangles_on_slice_face(point_cloud[0], progress_plot=progress_plot,
                                                                           plot_points=True)

    # For each contour in point_cloud
    for i in range(len(point_cloud) - 1):

        # Calculate the contour adjustment factor
        contour_adjustment = (0, 0, 0)
        if abs((contour_adjustment_magnitudes[i] / median_contour_adjustment_magnitude) - 1) <= 1.0:
            contour_adjustment = (contour_adjustment_vectors[i][0], 0, contour_adjustment_vectors[i][2])

        # Create lists of the unpaired points on this and the next contour
        unpaired_points_on_this_contour: list = deepcopy(point_cloud[i])
        unpaired_points_on_next_contour: list = deepcopy(point_cloud[i + 1])

        # If plotting the progress, plot the next contour
        plot_contour_points(point_cloud[i + 1], progress_plot, POINT_COLOR)

        # Find the first common pair of points
        previous_common_pair = find_common_pair_of_closest_points(unpaired_points_on_this_contour,
                                                                  unpaired_points_on_next_contour,
                                                                  point_cloud[i], point_cloud[i + 1],
                                                                  contour_adjustment)

        # Update the corresponding lists and plot it if necessary
        clean_up_line(previous_common_pair, unpaired_points_on_this_contour, unpaired_points_on_next_contour,
                      shapes_that_have_been_plotted, progress_plot)

        # Define a list of common lines that have not been used to create two triangles
        partially_available_common_lines = []
        add_new_partially_available_common_pair(partially_available_common_lines, previous_common_pair, 0)

        common_pairs_previously_paired: list = []

        # Define a loop counter for diagnostics
        loop_counter = 0

        # While points have not been paired,
        while len(unpaired_points_on_this_contour) + len(unpaired_points_on_next_contour) > 0 or \
                len(partially_available_common_lines) > 0:

            if i == 3 and (loop_counter == 60):
                print('BREAK!')

            # Reset the flag indicating if either line was used
            either_common_line_used = False

            # Create a flag indicating that a new common pair has not been created yet
            new_common_pair_created = False

            # Define the default action between two of common pairs
            action_to_complete = None

            # If there is more than one partially available common pair
            if len(partially_available_common_lines) > 1 and loop_counter > 5:

                # Find the pair of common pairs with the smallest index distance
                all_pair_combinations_with_index_distance = \
                    find_closest_pair_of_common_pairs(partially_available_common_lines, point_cloud[i],
                                                      point_cloud[i + 1])

                # Attempt to use every unique combination of partially available common lines,
                for index_distances, previous_common_pair, new_common_pair in all_pair_combinations_with_index_distance:

                    # Define a flag to determine if the pair of common pairs has been not used before
                    this_pair_has_not_been_used_before = True

                    # Search through all previous pairs of common pairs,
                    for pair_of_pairs in common_pairs_previously_paired:

                        # If the current pair has been used before, set the flag and break out of the loop
                        if previous_common_pair in pair_of_pairs and new_common_pair in pair_of_pairs:
                            this_pair_has_not_been_used_before = False
                            break

                    # If the previous and new common pair have not been used before,
                    if this_pair_has_not_been_used_before:

                        # Determine which action fits the distance between pairs
                        action_to_complete, comparison_index_distances = determine_line_action(index_distances)

                        if action_to_complete is not None:

                            # Break out of this loop and use the current data going forward
                            break

            # If an action cannot be completed using two partially available common pairs,
            if action_to_complete is None:

                if len(unpaired_points_on_this_contour) == 0 or len(unpaired_points_on_next_contour) == 0:
                    print('BREAK!')
                # Find the next common pair of points
                new_common_pair = find_common_pair_of_closest_points(unpaired_points_on_this_contour,
                                                                     unpaired_points_on_next_contour,
                                                                     point_cloud[i], point_cloud[i + 1],
                                                                     contour_adjustment)

                # Flag that a new previous common pair has not been found yet
                new_previous_common_pair_not_found: bool = True

                # Make a temporary copy of the next contours
                temp_next_contour: list = copy(point_cloud[i + 1])

                # While a previous contour has not been found,
                while new_previous_common_pair_not_found:

                    # Try to find the closest common pair to the new common pair,
                    try:
                        previous_common_pair, index_distances = find_closest_common_pair(
                            partially_available_common_lines,
                            new_common_pair,
                            point_cloud[i],
                            temp_next_contour)
                        new_previous_common_pair_not_found = False

                    # If one does not exist, remove the point from the next contour that was found to be the closest
                    except:
                        temp_next_contour.pop(new_common_pair.point_2.index)

                # Update the flag
                new_common_pair_created = True

                # Plot the new line for clarity
                plot_single_line(new_common_pair, shapes_that_have_been_plotted, progress_plot)

                # Determine which action fits the distance between pairs
                action_to_complete, comparison_index_distances = determine_line_action(index_distances)

            # If the common pairs are parallel and next to one another,
            if action_to_complete == LINES_FORM_QUADRILATERAL:

                # Set the flag indicating the common lines were used
                either_common_line_used = True

                # Create new triangles from the two parallel lines
                new_triangles = create_triangles_from_quadrilateral(previous_common_pair, new_common_pair)

                # Add them to the list of triangles on the shape
                list_of_triangles = list_of_triangles + [t.convert_to_vertices() for t in new_triangles]

                # Update the corresponding lists and plot if necessary
                clean_up_generated_triangles(new_triangles, unpaired_points_on_this_contour,
                                             unpaired_points_on_next_contour, shapes_that_have_been_plotted,
                                             progress_plot)

            # If the common pairs are adjacent on one contour but separated by one or more points on the other,
            elif action_to_complete == NO_GAP_AND_GAP:

                # Set the flag indicating the common lines were used
                either_common_line_used = True

                # Determine the directionality of the gap filling
                directionality = int(index_distances[0] / comparison_index_distances[0])

                # Define a variable for storing how many lines are connected to the previous and new common pair
                lines_connected_to_each_common = [[], []]

                # Determine which contour has the gap between the common pairs
                contour_with_gap = comparison_index_distances.index(max(comparison_index_distances))

                # Calculate the indices of the points between the two common pairs
                points_contained_in_the_gap = wrapping_range(
                    previous_common_pair.get_data_of_point_given_contour(contour_with_gap, INDEX_DATA) + directionality,
                    new_common_pair.get_data_of_point_given_contour(contour_with_gap, INDEX_DATA),
                    point_cloud[i + contour_with_gap])

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
                    if directionality > 0:
                        lines_connected_to_each_common[closest_point_index].insert(
                            len(lines_connected_to_each_common[closest_point_index]), new_line)
                    else:
                        lines_connected_to_each_common[closest_point_index].insert(0, new_line)

                    # Plot the new line if necessary
                    plot_single_line(new_line, shapes_that_have_been_plotted, progress_plot)

                # Define a list to store the triangles that will be generated
                new_triangles = []

                # Change the indices to work with based on the directionality
                if directionality > 0:
                    working_indices = (0, -1)
                    working_offsets = (1, -1)
                else:
                    working_indices = (-1, 0)
                    working_offsets = (-1, 1)

                # For each set of lines found previously and its associated common pair, create triangles
                for list_of_lines, associated_common, \
                    working_index, working_offset in zip(lines_connected_to_each_common,
                                                         (previous_common_pair,
                                                          new_common_pair),
                                                         working_indices,
                                                         working_offsets):

                    # If there are lines connected to this common,
                    if len(list_of_lines) > 0:

                        # Append a new triangle that contains the associated common and the first line
                        new_triangles.append(
                            Triangle(associated_common, list_of_lines[working_index],
                                     SingleContourLine(associated_common.get_point_from_contour(contour_with_gap),
                                                       list_of_lines[working_index].get_point_from_contour(
                                                           contour_with_gap),
                                                       contour_with_gap,
                                                       CONTOUR_LINE)))

                        # Make more triangles as long as more than one line is connected to the common
                        while len(list_of_lines) > 1:
                            # Append a new triangle that contains the first and second lines connected to the common
                            new_triangles.append(
                                Triangle(list_of_lines[working_index],
                                         list_of_lines[working_index + working_offset],
                                         SingleContourLine(
                                             list_of_lines[working_index].get_point_from_contour(contour_with_gap),
                                             list_of_lines[working_index + working_offset].get_point_from_contour(
                                                 contour_with_gap),
                                             contour_with_gap,
                                             CONTOUR_LINE)))

                            # Pop out the first line in the list to ensure that the looping ends
                            list_of_lines.pop(working_index)

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
                new_triangles = new_triangles + create_triangles_from_quadrilateral(previous_line_to_use,
                                                                                    next_line_to_use)

                # Update the corresponding lists and plot if necessary
                clean_up_generated_triangles(new_triangles, unpaired_points_on_this_contour,
                                             unpaired_points_on_next_contour, shapes_that_have_been_plotted,
                                             progress_plot)

                # Add the triangles to the master list of triangles
                list_of_triangles = list_of_triangles + [t.convert_to_vertices() for t in new_triangles]

            # If the common pairs are separated by one point on contour and one or more points on the other contour,
            elif action_to_complete == SINGLE_POINT_GAP_AND_GAP:

                # Determine the directionality of the gap filling
                directionality = int(index_distances[0] / comparison_index_distances[0])

                # If there is only one point between the common pairs on each side,
                if index_distances[0] == index_distances[1]:

                    # Create a new common pair between the only two available points
                    common_pair_in_gap = CrossContourLine(
                        Point(previous_common_pair.get_data_of_point_given_contour(THIS_CONTOUR,
                                                                                   INDEX_DATA) + directionality,
                              point_cloud[i][previous_common_pair.get_data_of_point_given_contour(THIS_CONTOUR,
                                                                                                  INDEX_DATA) +
                                             directionality]),
                        Point(previous_common_pair.get_data_of_point_given_contour(NEXT_CONTOUR,
                                                                                   INDEX_DATA) + directionality,
                              point_cloud[i + 1][previous_common_pair.get_data_of_point_given_contour(NEXT_CONTOUR,
                                                                                                      INDEX_DATA) +
                                                 directionality]), COMMON_CLOSEST_POINT_COLOR)

                # Find a new common pair by finding the closest pair of points,
                else:

                    # Determine which contour has the smallest gap
                    contour_with_small_gap = index_distances.index(min(index_distances))
                    contour_with_large_gap = index_distances.index(max(index_distances))

                    # Define two list to store the points in each gap
                    small_gap_points = []
                    large_gap_points = []

                    # Find the actual points in each gap
                    for contour, list_of_points_in_gap in zip([contour_with_small_gap, contour_with_large_gap],
                                                              [small_gap_points, large_gap_points]):
                        # Find the indices of the points in the small gap
                        temp_gap_indices = wrapping_range(
                            previous_common_pair.get_data_of_point_given_contour(contour, INDEX_DATA) +
                            directionality,
                            new_common_pair.get_data_of_point_given_contour(contour, INDEX_DATA),
                            point_cloud[i + contour])

                        # Find the actual values of the points in the small gap
                        for index in temp_gap_indices:
                            list_of_points_in_gap.append(point_cloud[i + contour][index])

                    # Determine which gap is on which contour
                    if contour_with_small_gap == THIS_CONTOUR and contour_with_large_gap == NEXT_CONTOUR:
                        points_on_gap_in_this_contour = small_gap_points
                        points_on_gap_in_next_contour = large_gap_points
                    elif contour_with_large_gap == THIS_CONTOUR and contour_with_small_gap == NEXT_CONTOUR:
                        points_on_gap_in_this_contour = large_gap_points
                        points_on_gap_in_next_contour = small_gap_points
                    else:
                        raise Exception('Contours were not recognized')

                    # Create a new common pair between these points
                    common_pair_in_gap = find_common_pair_of_closest_points(points_on_gap_in_this_contour,
                                                                            points_on_gap_in_next_contour,
                                                                            point_cloud[i], point_cloud[i + 1],
                                                                            contour_adjustment)

                # Add the new common found in the gap to the list of partially available commons
                add_new_partially_available_common_pair(partially_available_common_lines, common_pair_in_gap, 0)

                # Save both the common pair found in the gap and the new common pair
                for common_pair in [new_common_pair, common_pair_in_gap]:
                    clean_up_line(common_pair, unpaired_points_on_this_contour, unpaired_points_on_next_contour,
                                  shapes_that_have_been_plotted, progress_plot)

            else:

                # Update the corresponding lists and plot it if necessary
                clean_up_line(new_common_pair, unpaired_points_on_this_contour, unpaired_points_on_next_contour,
                              shapes_that_have_been_plotted, progress_plot)

            # Increase the use count of the previous common pair and remove it if needed
            if either_common_line_used:

                # Save that the current pair of commons has been paired
                common_pairs_previously_paired.append((previous_common_pair, new_common_pair))

                indices_to_pop = []
                for index, common_pair in enumerate(partially_available_common_lines):
                    if common_pair[OBJECT] == previous_common_pair or \
                            (not new_common_pair_created and common_pair[OBJECT] == new_common_pair):
                        common_pair[NUM_TIMES_USED] = common_pair[NUM_TIMES_USED] + 1
                        if common_pair[NUM_TIMES_USED] >= 2:
                            indices_to_pop.append(index)
                while len(indices_to_pop) > 0:
                    partially_available_common_lines.pop(indices_to_pop.pop(-1))

            # Add the newly created common pair to the list of partially filled common pairs
            if new_common_pair_created:
                add_new_partially_available_common_pair(partially_available_common_lines, new_common_pair,
                                                        either_common_line_used)

            loop_counter = loop_counter + 1

        print('All points found')

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


def find_closest_pair_of_common_pairs(list_of_common_pairs: list, this_contour: list, next_contour: list):
    all_combinations = []
    # smallest_index_distance = (10 ** 9, 10 ** 9)
    # closest_pair_previous = None
    # closest_pair_next = None

    indices_to_use = list(range(len(list_of_common_pairs)))

    # For each common pair excluding the last one,
    while len(indices_to_use) > 1:

        first_index = indices_to_use.pop(0)

        # # Create a new list of common pairs that does not include the one being searched for
        # modified_list_of_common_pairs = copy(list_of_common_pairs)
        # modified_list_of_common_pairs.pop(i)

        # For each common pair in the modified list,
        for second_index in indices_to_use:
            common_pair = list_of_common_pairs[first_index][OBJECT]
            second_common_pair = list_of_common_pairs[second_index][OBJECT]

            # Calculate the index distance between the two points
            new_index_distance = calc_index_distances_between_common_pairs(common_pair, second_common_pair,
                                                                           this_contour, next_contour)

            # Save each combination and the distance between each common
            all_combinations.append((new_index_distance, common_pair, second_common_pair))

            # # If the new distance is smaller than the previous smallest distance,
            # if abs(sum(new_index_distance)) < abs(sum(smallest_index_distance)):
            #     # Save the new distance and the two pairs
            #     smallest_index_distance = new_index_distance
            #     closest_pair_previous = common_pair
            #     closest_pair_next = second_common_pair

    # Sort the combinations by index distance
    all_combinations.sort(key=lambda data: sum(data[0]))

    # Return the index distance and the pairs
    return all_combinations


def find_closest_common_pair(list_of_common_pairs: list, reference_common: CrossContourLine,
                             this_contour: list, next_contour: list):
    positive_distances = []
    positive_common_pairs = []
    negative_distances = []
    negative_common_pairs = []

    for common_pair in list_of_common_pairs:
        temp_distance = calc_index_distances_between_common_pairs(common_pair[OBJECT], reference_common,
                                                                  this_contour, next_contour)
        if sum(temp_distance) > 0:
            positive_distances.append(temp_distance)
            positive_common_pairs.append(common_pair[OBJECT])
        elif sum(temp_distance) < 0:
            negative_distances.append(temp_distance)
            negative_common_pairs.append(common_pair[OBJECT])
        else:
            pass

    # Calculate the smallest distances in both directions
    try:
        smallest_positive_distance = min(positive_distances)
    except ValueError:
        smallest_positive_distance = None
    try:
        smallest_negative_distance = max(negative_distances)
    except ValueError:
        smallest_negative_distance = None

    try:
        if sum(smallest_positive_distance) <= abs(sum(smallest_negative_distance)):
            return positive_common_pairs[
                       positive_distances.index(smallest_positive_distance)], smallest_positive_distance
        else:
            return negative_common_pairs[
                       negative_distances.index(smallest_negative_distance)], smallest_negative_distance
    except TypeError:
        if smallest_positive_distance is None and smallest_negative_distance is not None:
            return negative_common_pairs[
                       negative_distances.index(smallest_negative_distance)], smallest_negative_distance
        elif smallest_positive_distance is not None and smallest_negative_distance is None:
            return positive_common_pairs[
                       positive_distances.index(smallest_positive_distance)], smallest_positive_distance
        else:
            raise Exception('No closest points could be found.')


def add_new_partially_available_common_pair(partially_available_common_pairs: list,
                                            common_pair: CrossContourLine, initial_usage: int = 1):
    partially_available_common_pairs.append([common_pair, initial_usage])


def determine_line_action(index_distances: tuple):
    # Create a new version of the index distances with absolute values for selecting connection approach
    comparison_index_distances = tuple([abs(x) for x in index_distances])

    # If the common pairs are parallel and next to one another,
    if sum(comparison_index_distances) == 2:
        return LINES_FORM_QUADRILATERAL, comparison_index_distances

    # If the common pairs are adjacent on one contour but separated by one or more points on the other,
    elif max(comparison_index_distances) > min(comparison_index_distances) == 1:
        return NO_GAP_AND_GAP, comparison_index_distances

    # If the common pairs are separated by one point on contour and one or more points on the other contour,
    elif max(index_distances) >= min(index_distances) == 2:
        return SINGLE_POINT_GAP_AND_GAP, comparison_index_distances

    # Otherwise return no action
    else:
        return None, comparison_index_distances

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
