# Import standard packages
from numpy import array

# Import custom packages
from thyroid_ultrasound_imaging_support.VolumeGeneration.calc_index_distance import calc_index_distance
from thyroid_ultrasound_imaging_support.VolumeGeneration.calc_norm import calc_norm


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


