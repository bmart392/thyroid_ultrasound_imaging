"""
File containing the find_external_points function definition.
"""

# Import standard python packages
from typing import List
from numpy import ndarray
from matplotlib.pyplot import axes

# Import custom python packages
from thyroid_ultrasound_imaging_support.VolumeGeneration.LineSegment import LineSegment
from thyroid_ultrasound_imaging_support.VolumeGeneration.wrap_increment import wrap_increment


def find_external_points_v2(list_of_points_on_contour,
                            return_indices: bool = True,
                            visualization_plot: axes = None) -> list:
    """
    Finds the external points of a convex hull of the given 2D contour.

    Parameters
    ----------
    list_of_points_on_contour : List[tuple] or List[list] or ndarray
        A collection of 2D points.
    return_indices :
        If true, the function will return the indices rather than the point values
    visualization_plot :

    Returns
    -------
    list :
        Returns the points as either their values or their indices in the source list
    """

    # Save the number of points in the contour for future use
    num_points = len(list_of_points_on_contour)

    # Stop if the contour does not have enough points
    if num_points < 3:
        raise Exception('Contour is too short')

    # Calculate the upper limit of how many boundaries to try making before giving up
    external_boundary_failure_limit = int(num_points / 2)

    # Calculate the maximum number of loops allowed before exiting to prevent being caught in an infinite loop
    loop_counter_maximum = 2 * num_points

    # Create two lists of indices to allow for simpler looping
    contour_point_loop_range = range(num_points)
    contour_point_loop_indexes = list(contour_point_loop_range) + [0]

    # Define a list to store every existing_boundary in the contour
    all_boundaries = []

    # Create a boundary between each consecutive point
    for i in contour_point_loop_range:
        all_boundaries.append(LineSegment(list_of_points_on_contour[contour_point_loop_indexes[i]],
                                          list_of_points_on_contour[contour_point_loop_indexes[i + 1]]))

    # For each point in the contour,
    for search_start_index in contour_point_loop_range:

        # Redefine the lists used to store internal and external points
        external_points = []
        external_point_indices = []
        internal_point_indices = []

        # Redefine the counters for how many times points and boundaries have failed
        num_failed_external_boundaries = 0
        num_failed_points = 0

        # Initialize the values for the first two points to create a boundary from
        first_point_in_boundary = search_start_index
        second_point_in_boundary = search_start_index

        # Redefine the loop counter
        loop_counter = 0

        # While not stuck in an infinite loop,
        while loop_counter < loop_counter_maximum:

            # Increment the second point in the boundary
            second_point_in_boundary = wrap_increment(second_point_in_boundary, num_points)

            # If the new second point is known to be internal, restart the loop to select a new one
            if second_point_in_boundary in internal_point_indices:
                continue

            # Create a new boundary
            new_boundary = LineSegment(list_of_points_on_contour[first_point_in_boundary],
                                       list_of_points_on_contour[second_point_in_boundary])

            # If the boundary is external,
            if new_boundary.check_if_boundary_is_external(all_boundaries):

                # If the first vertex is not already in the list, add it
                if new_boundary.vertices[0] not in external_points:
                    external_points.append(new_boundary.vertices[0])
                    external_point_indices.append(first_point_in_boundary)

                # If the second vertex is not already in the list, add it
                if new_boundary.vertices[1] not in external_points:
                    external_points.append(new_boundary.vertices[1])
                    external_point_indices.append(second_point_in_boundary)

                # If at least three external points have been found and
                # the boundary ends where the first boundary started
                if len(external_points) > 2 and \
                        list_of_points_on_contour[second_point_in_boundary] == external_points[0]:

                    # Return the appropriate list
                    if return_indices:
                        return external_point_indices
                    else:
                        return external_points

                # Set the first point of the next boundary as the ending point for the current boundary
                first_point_in_boundary = second_point_in_boundary

                # Reset the failure counter
                num_failed_external_boundaries = 0

                # Set the color of the boundary line
                boundary_line_color = 'green'

            # Otherwise, increment the failure counter
            else:
                num_failed_external_boundaries = num_failed_external_boundaries + 1

                # Set the color of the boundary line
                boundary_line_color = 'red'

            # If the current first boundary point is not valid,
            if num_failed_external_boundaries > external_boundary_failure_limit:

                # If the recovery process has failed less than 10 times and if there are currently external points,
                if num_failed_points < 10 and len(external_points) > 0:

                    # Reset the counter for the number of failed boundaries
                    num_failed_external_boundaries = 0

                    # Increment the number of points that were tried and failed
                    num_failed_points = num_failed_points + 1

                    # Add the last external point to the list of points that is known to be internal
                    internal_point_indices.append(external_point_indices[-1])

                    # Remove the last external point because it was not actually external
                    external_points.pop(-1)
                    external_point_indices.pop(-1)

                    # Set the new first point in the boundary to be the now last known external point
                    first_point_in_boundary = external_point_indices[-1]

                    # Reset the second point in the boundary
                    second_point_in_boundary = first_point_in_boundary

                    loop_counter = 0

                # Otherwise, start the search over
                else:
                    break

            # If a plot was given, plot the new boundary with the appropriate color
            if visualization_plot is not None:
                temp_x, temp_y = new_boundary.get_vertices_as_x_and_y()
                visualization_plot.plot(temp_x, temp_y, c=boundary_line_color)

            # Update the loop counter
            loop_counter = loop_counter + 1

    return []


if __name__ == '__main__':
    # Define a test contour
    test_contour = [(1, 2), (3, 2), (5, 2), (6, 3),
                    (7, 2), (8, 4), (8, 9), (7, 10),
                    (6, 9), (6, 8), (6, 6), (6, 5),
                    (5, 5), (4, 5), (3, 6), (2, 8),
                    (1, 7), (1, 4)]

    print(find_external_points_v2(test_contour, False))
