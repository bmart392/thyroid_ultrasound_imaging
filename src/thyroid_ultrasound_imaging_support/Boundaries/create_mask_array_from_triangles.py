"""
Contains create_mask_array_from_triangles function.
"""
# Import from standard packages
from numpy import uint8, zeros

# Import custom objects
from thyroid_ultrasound_imaging_support.Boundaries.BoundingSet import BoundingSet


def create_mask_array_from_triangles(list_of_foreground_triangles: list, image_shape: tuple):
    """
    Create a numpy array mask showing only the foreground of the image using triangles.

    Parameters
    ----------
    list_of_foreground_triangles
        a list of triangles, passed in as sets of 3 coordinates.
    image_shape
        a tuple of the shape of the image.
    """
    # Create blank mask array
    selected_foreground_mask = zeros(image_shape, uint8)

    # Define a variable to store the bounding sets within
    foreground_bounding_sets = []

    # Create all foreground bounding sets from their equivalent triangle
    for foreground_triangle in list_of_foreground_triangles:
        foreground_bounding_sets.append(BoundingSet(foreground_triangle))

    # Define variables to store the min and max x and y coordinates
    min_y = 100000000
    max_y = -100000000
    min_x = 100000000
    max_x = -100000000

    # Find the min and max x and y coordinates in the foreground triangles
    for triangle in list_of_foreground_triangles:
        for coordinate in triangle:
            if coordinate[0] > max_x:
                max_x = coordinate[0]
            if coordinate[0] < min_x:
                min_x = coordinate[0]
            if coordinate[1] > max_y:
                max_y = coordinate[1]
            if coordinate[1] < min_y:
                min_y = coordinate[1]

    # Iterate through each element of the mask that might be part of the foreground
    for xx in range(min_x, max_x):
        for yy in range(min_y, max_y):

            # Check if the given element is contained within the foreground
            for foreground_set in foreground_bounding_sets:
                if foreground_set.is_point_within_set([xx, yy]):

                    # Update the mask value accordingly
                    selected_foreground_mask[yy][xx] = uint8(1)

                    # Stop looping once it has been found in one boundary
                    break

    # Return the resulting mask
    return selected_foreground_mask
