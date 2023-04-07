"""
Contains get_threshold_values_from_triangle function.
"""
# Import from standard packages
from numpy import array
from scripts.Boundaries.BoundingSet import BoundingSet


def get_threshold_values_from_triangles(list_of_triangles: list, image: array):
    """
    Calculate the values needed to threshold filter an image from a set of triangles.

    Parameters
    ----------
    list_of_triangles
        a list of triangles, passed in as sets of 3 coordinates.
    image
        a numpy array representing the image.
    """
    # TODO Get this function to actually work
    lowest_number = 255
    highest_number = 0

    bounding_sets = []

    for triangle in list_of_triangles:
        bounding_sets.append(BoundingSet(triangle))

    for y in range(image.shape[0]):
        for x in range(image.shape[1]):

            for bounding_set in bounding_sets:
                if bounding_set.is_point_within_set((x, y)):
                    if image[y][x][0] > highest_number:
                        highest_number = image[y][x][0]
                    elif image[y][x][0] < lowest_number:
                        lowest_number = image[y][x][0]
                    break

    return lowest_number, highest_number
