"""
Contains get_threshold_values_from_triangle function.
"""
# Import from standard packages
from numpy import array, std, median
from scripts.Boundaries.BoundingSet import BoundingSet


def get_threshold_values_from_triangles(list_of_triangles: list, image: array,
                                        num_standard_deviations: float = 2,
                                        display_result: bool = False) -> tuple:
    """
    Calculate the values needed to threshold filter an image from a set of triangles.

    Parameters
    ----------
    list_of_triangles
        a list of triangles, passed in as sets of 3 coordinates.
    image
        a numpy array representing the image.
    num_standard_deviations
        the number of standard deviations away from the median used to calculate the upper and lower threshold values.
    display_result
        select to print the result of this function to the terminal. Useful for finding values to hardcode.
    """

    # Define a list to hold the bounding sets containing the area to use to set the threshold values
    bounding_sets = []

    # Define a list of all values contained in the bounding sets
    values_list = []

    # For each triangle created from the user selection, create an equivalent bounding set
    for triangle in list_of_triangles:
        bounding_sets.append(BoundingSet(triangle))

    # Iterate through each pixel in the image to find those contained within any single bounding box
    for y in range(image.shape[0]):
        for x in range(image.shape[1]):

            # If the image is in one of the bounding sets
            for bounding_set in bounding_sets:
                if bounding_set.is_point_within_set([x, y]):
                    a = image[y][x][0]
                    # Save the value and move to the next pixel
                    values_list.append(image[y][x][0])
                    break

    # Calculate the median value of all values found
    median_value = median(values_list)

    # Calculate the standard deviation of all values found
    standard_deviation = std(values_list)

    # Calculate the median plus/minus the given number of standard deviations
    result = (int(median_value - (num_standard_deviations * standard_deviation)),
              int(median_value + (num_standard_deviations * standard_deviation)))

    if display_result:
        print("Thresholding Values (lower bound, upper bound):")
        print(result)

    # Return the median plus/minus the given number of standard deviations
    return result
