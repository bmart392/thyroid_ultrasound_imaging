"""
File containing the find_external_points function definition.
"""

# Import standard python packages
from typing import List
from copy import copy
from numpy import ndarray, delete

# Import custom python packages
from thyroid_ultrasound_imaging_support.Boundaries.BoundaryPrimitive import BoundaryPrimitive, EdgeException
from thyroid_ultrasound_imaging_support.VolumeGeneration.wrapping_range import wrapping_range


def find_external_points(list_of_points_on_contour,
                         return_indices: bool = True) -> list:
    """
    Finds the external points of a 2D contour.

    Parameters
    ----------
    list_of_points_on_contour : List[tuple] or List[list] or ndarray
        A collection of 2D points.
    return_indices :
        If true, the function will return the indices rather than the point values

    Returns
    -------
    list :
        Returns the points as either their values or their indices in the source list
    """

    # Check if the input type is an array
    list_is_array = isinstance(list_of_points_on_contour, ndarray)

    # Define a list to store the points contained externally
    external_points = []

    # Define a variable in which to store the index of the first point found on a correct boundary primitive
    first_found_point_index = None

    # Define an index for tracking the index of the first point in each boundary primitive
    i = 0

    # Define an index for tracking the index of the second point in each boundary primitive
    j = 0

    # Define a loop counter to ensure that the function will stop eventually
    loop_counter = 0

    # While the function has not completed or has not iterated more times than 110% of the list length,
    while loop_counter < int(len(list_of_points_on_contour) * 1.1):

        # Define the first point in the boundary primitive as the second point found in the previous boundary primitive
        first_point = list_of_points_on_contour[i]

        # For each index in the contour between 1 point after the first point and halfway around the contour,
        for j in wrapping_range(i + 1, i + 1 + int(len(list_of_points_on_contour) / 2), list_of_points_on_contour):

            # Define the second point of the boundary primitive
            second_point = list_of_points_on_contour[j]

            # Create a boundary primitive between the two points
            temp_bounding_set = BoundaryPrimitive(first_point, second_point)

            # Define a variable to store the number of points either inside or outside the boundary
            running_result = 0

            # Define a variable to store the number of points that were found to be on the boundary
            num_points_on_boundary = 0

            # Create a list of points that does not include the points used to define the boundary
            list_of_points_on_contour_excluding_current_point = copy(list_of_points_on_contour)
            if list_is_array:
                delete(list_of_points_on_contour_excluding_current_point, max(i, j))
                delete(list_of_points_on_contour_excluding_current_point, min(i, j))
            else:
                list_of_points_on_contour_excluding_current_point.pop(max(i, j))
                list_of_points_on_contour_excluding_current_point.pop(min(i, j))

            # For every point in the contour,
            for k, test_point in enumerate(list_of_points_on_contour_excluding_current_point):

                # Determine if it is inside, outside, or on the boundary
                try:
                    running_result = running_result + temp_bounding_set.point_within_boundary(test_point,
                                                                                              throw_edge_exception=True)
                except EdgeException:
                    num_points_on_boundary = num_points_on_boundary + 1

            # If all the points were inside the boundary, except for those on the boundary, or all of the points
            # were outside the boundary,
            if running_result == (len(list_of_points_on_contour_excluding_current_point) - num_points_on_boundary) \
                    or running_result == 0:

                # If the second point of the boundary is not the same as the first point in the first boundary found,
                if j != first_found_point_index:

                    # Add the second point to the list of external points
                    if return_indices:
                        external_points.append(i)
                    else:
                        external_points.append(first_point)

                    # If this is the first boundary to be found,
                    if first_found_point_index is None:

                        # Save the index of the first point of the boundary
                        first_found_point_index = j

                    # For the next iteration, set the index of the first point of the next boundary to be the index of
                    # second point of this boundary
                    i = j

                    # Break out of the for loop
                    break

                # Otherwise, exit the function because all points have been found
                else:
                    return external_points

        # If a boundary was not found (and therefore the indices i and j were not set),
        if i != j:

            # Try again with the next point in the boundary as the first point,
            i = i + 1

            # Loop the index back to the beginning of the list if needed
            if i > (len(list_of_points_on_contour) - 1):
                i = i - len(list_of_points_on_contour)

        # Update the iteration counter
        loop_counter = loop_counter + 1

    # Raise an exception if the while loop exited due to the iteration count
    raise MaximumIterationsExceeded('Iterations completed: ' + str(loop_counter))


class MaximumIterationsExceeded(Exception):

    def __init__(self, message):
        self.message = message
        super().__init__(self.message)


if __name__ == '__main__':
    # Define a test contour
    test_contour = [(2, 2), (3, -2), (1, -3),
                    (-1, -1), (-5, 1), (-1, 1), (-3, 3),
                    (-1, 5), (-2, 6), (3, 3), ]

    print(find_external_points(test_contour, False))
