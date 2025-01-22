"""
File containing the find_external_points function definition.
"""

# Import standard python packages
from typing import List
from copy import copy
from numpy import ndarray, delete, ceil

# Import custom python packages
from thyroid_ultrasound_imaging_support.Boundaries.BoundaryPrimitive import BoundaryPrimitive, EdgeException
from thyroid_ultrasound_imaging_support.VolumeGeneration.wrapping_range import wrapping_range
from thyroid_ultrasound_imaging_support.VolumeGeneration.MaximumIterationsExceeded import MaximumIterationsExceeded
from thyroid_ultrasound_imaging_support.Boundaries.BoundarySegment import BoundarySegment
from thyroid_ultrasound_imaging_support.VolumeGeneration.LineSegment import LineSegment
from thyroid_ultrasound_imaging_support.VolumeGeneration.Line import Line


def find_external_points(list_of_points_on_contour,
                         return_indices: bool = True) -> list:
    """
    Finds the external points of a convex hull of the given 2D contour.

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

    # TODO - High - What if: 1. start with point 0, 2. create line segment between 0 and i. 3. increase i until a valid boundary is found 4. add that valid boundary 5. if no valid boundary can be found, that point is bad and try from the next point. 6. Continue until the first successful point is reached again
    # To deal with starting on a section that is just bad, if you go around and reach a point where you cannot make a valid boundary with any point on the contour that has not been used already, then restart the whole process but from the next point after the last point that could be considered external. Repeat until a complete chain can be created or every point has been tried

    external_points = []
    external_boundaries = []

    # Create two lists of indices to allow for simpler looping
    contour_point_loop_range = range(len(list_of_points_on_contour))
    contour_point_loop_indexes = list(contour_point_loop_range) + [0]

    # Define a list to store every existing_boundary in the contour
    all_boundaries = []

    # Create every existing_boundary
    for i in contour_point_loop_range:
        all_boundaries.append(LineSegment(list_of_points_on_contour[contour_point_loop_indexes[i]],
                                          list_of_points_on_contour[contour_point_loop_indexes[i + 1]]))

    external_boundary_segments = []
    new_boundary_segment_primitives = []

    # For each existing_boundary and midpoint,
    for i in range(len(all_boundaries)):

        if i == 32 or i == 33:
            print('break!')

        # Check if the boundary is external,
        is_boundary_external = all_boundaries[i].check_if_boundary_is_external(all_boundaries)

        # If the existing_boundary is external,
        if is_boundary_external:

            # Add the current existing_boundary to the list of external boundaries
            external_boundaries.append(all_boundaries[i])

            # Add the first point, and its index, of this existing_boundary to the list of external points
            if all_boundaries[i].vertices[0] not in external_points:
                external_points.append(all_boundaries[i].vertices[0])
            if all_boundaries[i].vertices[1] not in external_points:
                external_points.append(all_boundaries[i].vertices[1])

            # Add the current existing_boundary segment to the list for the current existing_boundary segment
            new_boundary_segment_primitives.append(all_boundaries[i])

        # If the existing_boundary is not external or this is the last iteration,
        if not is_boundary_external or i == len(all_boundaries) - 1:
            if len(new_boundary_segment_primitives) > 0:
                external_boundary_segments.append(BoundarySegment(new_boundary_segment_primitives))
                new_boundary_segment_primitives = []

    # If external existing_boundary segments were found and the first and last segment share a vertex,
    if len(external_boundary_segments) > 1 and \
            external_boundary_segments[0].starting_vertex == external_boundary_segments[-1].ending_vertex:
        # Create a new combined existing_boundary segment
        new_boundary_segment = BoundarySegment(external_boundary_segments[-1].boundaries_contained_within +
                                               external_boundary_segments[0].boundaries_contained_within)

        # Remove the two segments
        external_boundary_segments.pop(-1)
        external_boundary_segments.pop(0)

        # Add the combined segment to the list
        external_boundary_segments.append(new_boundary_segment)

    print('Break!')

    # If no points were found to be external, raise an error
    if len(external_boundaries) == 0:
        raise Exception('This contour has not external boundaries')

    # If every point is external, return every point
    elif len(external_boundaries) == len(all_boundaries):
        if return_indices:
            return list(range(len(list_of_points_on_contour)))
        else:
            return list_of_points_on_contour
        pass

    # Otherwise, check that all external points are actually external,
    else:

        # # Define an index to determine where the first existing_boundary in the one of the existing_boundary segments is
        # first_boundary_primitive_index = 0
        #
        # # For each existing_boundary in the contour,
        # while first_boundary_primitive_index < len(all_boundaries):
        #
        #     # If the previous existing_boundary is not external and the current existing_boundary is,
        #     if all_boundaries[first_boundary_primitive_index - 1] not in external_boundaries and \
        #             all_boundaries[first_boundary_primitive_index] in external_boundaries:
        #
        #         # Exit so that the count can be used afterwards
        #         break
        #
        #     # Otherwise increment to check the next existing_boundary
        #     else:
        #         first_boundary_primitive_index = first_boundary_primitive_index + 1
        #
        # # Duplicate the list of all existing_boundary primitives
        # not_included_boundary_primitives = copy(all_boundaries)
        #
        # # Define a second index to increment around the contour to find each existing_boundary segment
        # k = first_boundary_primitive_index
        #
        # # Define a list to store each existing_boundary segment
        # boundary_segments = []
        #
        # # Identify each length of external points and create an object for it (stores the two end points, the number of points within, left-connection_valid, and right-connection-valid)
        #
        # # While some existing_boundary primitives have not been included in a existing_boundary segment,
        # while len(not_included_boundary_primitives) > 0:
        #
        #     # If the current existing_boundary is external
        #     if all_boundaries[k] in external_boundaries:
        #
        #         # Create a list to store the existing_boundary primitives in this existing_boundary segment
        #         new_boundary_segments = []
        #
        #         # While the current and each subsequent existing_boundary primitive is external,
        #         while all_boundaries[k] in external_boundaries:
        #
        #             # Add the current existing_boundary primitive to this existing_boundary segment
        #             new_boundary_segments.append(all_boundaries[k])
        #
        #             # Pop the current existing_boundary out of the list of not included boundaries
        #             not_included_boundary_primitives.pop(not_included_boundary_primitives.index(all_boundaries[k]))
        #
        #             # Increment to the next primitive
        #             k = k + 1
        #
        #             # Wrap around the end of the list as needed
        #             if k >= len(all_boundaries):
        #                 k = k - len(all_boundaries)
        #
        #             # Break out if the next existing_boundary primitive is the same as the starting existing_boundary primitive
        #             if k == first_boundary_primitive_index:
        #                 break
        #
        #         # Only create a new existing_boundary segment if connected existing_boundary primitives were found
        #         if len(new_boundary_segments) > 0:
        #             boundary_segments.append(BoundarySegment(new_boundary_segments))
        #
        #         # Break out if the next existing_boundary primitive is the same as the starting existing_boundary primitive
        #         if k == first_boundary_primitive_index:
        #             break
        #
        #     # Otherwise,
        #     else:
        #
        #         # Pop the current existing_boundary out of the list of not included boundaries
        #         not_included_boundary_primitives.pop(not_included_boundary_primitives.index(all_boundaries[k]))
        #
        #         # Go to the next existing_boundary primitive,
        #         k = k + 1
        #
        #         # Break out if the next existing_boundary primitive is the same as the starting existing_boundary primitive
        #         if k == first_boundary_primitive_index:
        #             break

        print('Break!')

        non_external_boundary_segments = []
        loop_range = range(len(external_boundary_segments))
        temp_external_boundary_segments = copy(external_boundary_segments)

        for i in loop_range:

            new_boundary_segment_is_external = False

            j = i + 1
            if j >= len(external_boundary_segments):
                j = j - len(external_boundary_segments)

            # TODO - High - IF there is only the ends of two boundary segments are only one off from each other but
            # the boundary between them is not external, try creating boundaries between 1 off from the start and end
            # respectively (up to 5 points from each side) until an external boundary is made, if that does not happen,
            # then give up on the boundary being external

            while not new_boundary_segment_is_external and j != i and len(temp_external_boundary_segments) > 1:

                # Create an existing_boundary between the end point of the i and i+1 existing_boundary segments
                new_boundary_segment_is_external = LineSegment(
                    external_boundary_segments[i].ending_vertex,
                    external_boundary_segments[j].starting_vertex).check_if_boundary_is_external(all_boundaries)
                    # check_if_boundary_is_external(None, all_boundaries,
                    #                                                              BoundaryPrimitive(
                    #                                                                  external_boundary_segments[
                    #                                                                      i].ending_vertex,
                    #                                                                  external_boundary_segments[
                    #                                                                      j].starting_vertex))

                # Increment the index in which to check
                j = j + 1
                if j >= len(external_boundary_segments):
                    j = j - len(external_boundary_segments)

            if not new_boundary_segment_is_external and j == i and len(temp_external_boundary_segments) > 1:
                non_external_boundary_segments.append(external_boundary_segments[i])
                temp_external_boundary_segments.pop(temp_external_boundary_segments.index(external_boundary_segments[i]))

        print('Break!')
        for boundary_segment in non_external_boundary_segments:
            for contained_vertex in boundary_segment.get_all_vertices():
                external_points.pop(external_points.index(contained_vertex))

        if return_indices:
            indices = []
            for point in external_points:
                indices.append(list_of_points_on_contour.index(point))
            return indices
        else:
            return external_points





if __name__ == '__main__':
    # Define a test contour
    test_contour = [(2, 2), (3, -2), (1, -1), (0, -2), (1, -3),
                    (-3, -1), (-1, 1), (-5, 1), (-3, 3),
                    (-1, 5), (-2, 6), (3, 3), ]

    print(find_external_points(test_contour, True))
