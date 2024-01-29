"""
Contains create_previous_image_mask_array_from_triangles function.
"""
# Import from standard packages
from numpy import uint8, ones, zeros
from cv2 import GC_PR_FGD, GC_BGD, GC_FGD, dilate

# Import custom objects
from thyroid_ultrasound_imaging_support.Boundaries.BoundingSet import BoundingSet
from thyroid_ultrasound_imaging_support.Boundaries.create_mask_array_from_triangles import create_mask_array_from_triangles


def create_previous_image_mask_array_from_triangles(list_of_background_triangles: list,
                                                    list_of_foreground_triangles: list,
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
    final_mask = ones(image_shape, uint8) * GC_PR_FGD
    selected_foreground_mask = zeros(image_shape, uint8)

    # Define two variables to store the bounding sets within
    background_bounding_sets = []
    foreground_bounding_sets = []

    # Create all background bounding sets from their equivalent triangle
    for background_triangle in list_of_background_triangles:
        background_bounding_sets.append(BoundingSet(background_triangle))

    # Create all foreground bounding sets from their equivalent triangle
    for foreground_triangle in list_of_foreground_triangles:
        foreground_bounding_sets.append(BoundingSet(foreground_triangle))

    # If both a foreground and background have been identified
    if len(background_bounding_sets) > 0:

        # Iterate through each element of the mask
        for xx in range(final_mask.shape[0]):
            for yy in range(final_mask.shape[1]):

                # Add flag to skip checking foreground if the location was in the background
                was_location_background = False

                # Check if the given element is contained within the background, foreground, or neither
                for background_set in background_bounding_sets:
                    if background_set.is_point_within_set([yy, xx]):

                        # Note that the location was found in a boundary
                        was_location_background = True

                        # Update the mask value accordingly
                        final_mask[xx][yy] = GC_BGD

                        # Stop looping once it has been found in one boundary
                        break

                if not was_location_background:
                    # Check if the given element is contained within the background, foreground, or neither
                    for foreground_set in foreground_bounding_sets:
                        if foreground_set.is_point_within_set([yy, xx]):

                            # Update the mask value accordingly
                            final_mask[xx][yy] = GC_FGD

                            # Stop looping once it has been found in one boundary
                            break

    # Otherwise, expand the foreground to find create the background
    else:

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

        # Dilate the foreground mask
        dilated_foreground_mask = dilate(selected_foreground_mask, ones((9, 9)), iterations=9, anchor=(4, 4))
        dilated_foreground_mask = dilated_foreground_mask.astype(uint8)

        # Create the background mask as the inverse of the dilated foreground mask and
        # define the values in the mask
        background_mask = (uint8(1) - dilated_foreground_mask) * (-GC_PR_FGD + GC_BGD)

        # Define the values in the foreground mask
        selected_foreground_mask = selected_foreground_mask * (-GC_PR_FGD + GC_FGD)

        # Calculate the final mask as a combination of the three masks
        final_mask = final_mask + background_mask + selected_foreground_mask
        final_mask = final_mask.astype(uint8)

    # Return the resulting mask
    return final_mask
