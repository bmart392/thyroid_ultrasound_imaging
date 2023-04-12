"""
Contains create_mask_array_from_triangles function.
"""
# Import from standard packages
from numpy import uint8, ones
from cv2 import GC_PR_BGD, GC_BGD, GC_FGD

# Import custom objects
from scripts.a_Boundaries.BoundingSet import BoundingSet


def create_mask_array_from_triangles(list_of_background_triangles: list, list_of_foreground_triangles: list,
                                     image_shape: tuple):
    """
    Create a numpy array mask showing the background and foreground of the image using triangles.

    Parameters
    ----------
    list_of_background_triangles
        a list of triangles, passed in as sets of 3 coordinates.
    list_of_foreground_triangles
        a list of triangles, passed in as sets of 3 coordinates.
    image_shape
        a tuple of the shape of the image.
    """
    # Create blank mask array and assign it all as probable background
    initial_mask = ones(image_shape, uint8) * GC_PR_BGD

    # Define two variables to store the bounding sets within
    background_bounding_sets = []
    foreground_bounding_sets = []

    # Create all background bounding sets from their equivalent triangle
    for background_triangle in list_of_background_triangles:
        background_bounding_sets.append(BoundingSet(background_triangle))

    # Create all foreground bounding sets from their equivalent triangle
    for foreground_triangle in list_of_foreground_triangles:
        foreground_bounding_sets.append(BoundingSet(foreground_triangle))

    # Iterate through each element of the mask
    for y in range(initial_mask.shape[0]):
        for x in range(initial_mask.shape[1]):

            # Add flag to skip checking foreground if the location was in the background
            was_location_background = False

            # Check if the given element is contained within the background, foreground, or neither
            for background_set in background_bounding_sets:
                if background_set.is_point_within_set((x, y)):

                    # Note that the location was found in a boundary
                    was_location_background = True

                    # Update the mask value accordingly
                    initial_mask[y][x] = GC_BGD

                    # Stop looping once it has been found in one boundary
                    break

            if not was_location_background:
                # Check if the given element is contained within the background, foreground, or neither
                for foreground_set in foreground_bounding_sets:
                    if foreground_set.is_point_within_set((x, y)):

                        # Update the mask value accordingly
                        initial_mask[y][x] = GC_FGD

                        # Stop looping once it has been found in one boundary
                        break

    # Return the resulting mask
    return initial_mask
