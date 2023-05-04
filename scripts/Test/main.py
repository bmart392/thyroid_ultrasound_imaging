"""
A script to test image filters and visualization methods.
"""
# Import standard packages
import numpy as np
import cv2
from copy import copy

# Import custom packages and functions
from thyroid_ultrasound_imaging.ImageData.ImageData import ImageData, import_images_as_image_data_objects

from thyroid_ultrasound_imaging.ImageFilter.ImageFilterThreshold import ImageFilterThreshold
from thyroid_ultrasound_imaging.ImageFilter.ImageFilterGrabCut import ImageFilterGrabCut
from thyroid_ultrasound_imaging.ImageFilter.ImageFilter import ImageFilter
from thyroid_ultrasound_imaging.ImageFilter.FilterConstants import COLOR_BGR

from thyroid_ultrasound_imaging.Visualization.Visualization import Visualization
from thyroid_ultrasound_imaging.Visualization.VisualizationConstants import *

from thyroid_ultrasound_imaging.UserInput.user_input_crop_coordinates import user_input_crop_coordinates
from thyroid_ultrasound_imaging.UserInput.user_input_polygon_points import user_input_polygon_points
from thyroid_ultrasound_imaging.UserInput.get_threshold_values_user_input import get_threshold_values_from_user_input

from thyroid_ultrasound_imaging.Boundaries.create_convex_triangles_from_points import create_convex_triangles_from_points
from thyroid_ultrasound_imaging.Boundaries.create_mask_array_from_triangles import create_mask_array_from_triangles

# Define image series names
SERIES_1: int = 1
SERIES_2: int = 2

if __name__ == '__main__':

    # Define several controls to ease testing
    capture_user_input = False  # True = define input, false = use values
    image_series_to_use = SERIES_2  # See above for options
    imaging_mode = IMG_CONTINUOUS  # Single or continuous

    # Define the visualizations to display for each filter
    list_of_visualizations = [
        [
            # SHOW_ORIGINAL,
            # SHOW_CROPPED,
            # SHOW_RECOLOR,
            # SHOW_BLUR,
            # SHOW_MASK,
            # SHOW_EXPANDED_MASK,
            SHOW_FOREGROUND,
            # SHOW_SURE_FOREGROUND,
            # SHOW_SURE_BACKGROUND,
            # SHOW_PROBABLE_FOREGROUND,
            # SHOW_INITIALIZED_MASK,
            SHOW_CENTROIDS_ONLY,
            # SHOW_CENTROIDS_CROSS_ONLY,

            # TODO Implement following mode
            # SHOW_MASK_OVERLAY,

            # TODO Implement following mode,
            # SHOW_CENTROIDS_CROSS_OVERLAY,

            # TODO Implement following mode
            # SHOW_MASK_CENTROIDS_CROSS_OVERLAY
        ],
        [
            # SHOW_ORIGINAL,
            # SHOW_CROPPED,
            # SHOW_BLUR,
            # SHOW_MASK,
            # SHOW_INITIALIZED_MASK,
            # SHOW_GRABCUT_USER_INITIALIZATION_0,
            # SHOW_EXPANDED_MASK,
            # SHOW_FOREGROUND,
            # SHOW_SURE_FOREGROUND,
            # SHOW_SURE_BACKGROUND,
            # SHOW_PROBABLE_FOREGROUND
            # SHOW_CENTROIDS_CROSS_ONLY,
            # SHOW_MASK_OVERLAY
        ]
    ]

    # Create the visualization objects
    visualizations = [Visualization(imaging_mode, list_of_visualizations[0],
                                    visualization_title="Grabcut Filter", num_cols_in_fig=3),
                      Visualization(imaging_mode, list_of_visualizations[1],
                                    visualization_title="Threshold Filter", num_cols_in_fig=3),
                      ]

    # Read in images from the image series as arrays of image data objects
    # Define saved image parameters for each series
    if image_series_to_use == SERIES_1:

        images = import_images_as_image_data_objects('Images/Series1', COLOR_BGR, 30)  # Read images from Series 1

        image_crop_coordinates = [[328, 67], [540, 578]]  # Series 1 - Slice 30
        list_of_points_for_background_polygon = [(21, 52), (183, 58), (173, 252), (27, 236)]  # Series 1 - Slice 30
        list_of_points_for_foreground_polygon = [(102, 292), (157, 318), (101, 345)]  # Series 1 - Slice 30

        thresholding_parameters = (100, 120)  # Series 1 - Slice 30

    elif image_series_to_use == SERIES_2:

        images = import_images_as_image_data_objects('Images/Series2', COLOR_BGR, 20)  # Read images from Series 2

        image_crop_coordinates = [[198, 197], [538, 494]]  # Series 2 - Slice 20
        list_of_points_for_background_polygon = [(2, 2), (334, 6), (321, 131), (163, 134), (49, 142),
                                                 (23, 179)]  # Series 2 - Slice 20
        list_of_points_for_foreground_polygon = [(27, 212), (128, 157), (58, 259)]  # Series 2 - Slice 20

        thresholding_parameters = (69, 124)  # Series 2 - Slice 20

    else:
        raise Exception("Series choice not recognized.")

    # Save an image for the user to annotate and to create the initial mask
    test_image = images[0]

    # If user input is required, overwrite the saved parameters for the series
    if capture_user_input:
        # Capture the image crop coordinates from the user
        image_crop_coordinates = user_input_crop_coordinates(test_image)

        # Capture the background of the image from the user
        list_of_points_for_background_polygon = user_input_polygon_points(test_image, "background",
                                                                          display_result=True)

        # Capture the foreground of the image from the user
        list_of_points_for_foreground_polygon = user_input_polygon_points(test_image, "foreground",
                                                                          display_result=True)

        # Use the user input to generate the thresholding values for the threshold filter
        thresholding_parameters = get_threshold_values_from_user_input(test_image,
                                                                       num_standard_deviations=1.75,
                                                                       display_result=True)

    # Create an ImageFilter object to crop the image so that the initial mask can be generated
    image_filter = ImageFilter(image_crop_coordinates=image_crop_coordinates)

    # Crop the test image
    image_filter.crop_image(test_image)

    # Convert the points of the background and foreground polygons to triangles
    list_of_background_triangles = create_convex_triangles_from_points(list_of_points_for_background_polygon)
    list_of_foreground_triangles = create_convex_triangles_from_points(list_of_points_for_foreground_polygon)

    # Use the polygons to generate the initial mask
    initial_mask = create_mask_array_from_triangles(list_of_background_triangles, list_of_foreground_triangles,
                                                    test_image.cropped_image.shape[:2])

    # Create the image filters
    image_filters = [ImageFilterGrabCut(previous_mask_array=initial_mask,
                                        analysis_mode=False,
                                        image_crop_included=True,
                                        image_crop_coordinates=image_crop_coordinates,
                                        increase_contrast=False
                                        ),
                     ImageFilterThreshold(analysis_mode=False,
                                          image_crop_included=True,
                                          increase_contrast=False,
                                          image_crop_coordinates=image_crop_coordinates,
                                          thresholding_parameters=thresholding_parameters),
                     ]

    # Initialize an empty array to hold the confidence map
    confidence_map = np.zeros(test_image.original_image.shape[:2])

    # Filter each image in the series
    for image_data in images:

        # Reduce the confidence of all cells in the map due to time having elapsed
        # TODO the temporal confidence value will need tuning
        confidence_map = confidence_map * .25  # This is the temporal confidence

        # For each image filter and visualization pair, segment the image and show the results
        for image_filter, visualization in zip(image_filters, visualizations):

            # Create a temporary copy of the image data
            temp_image_data: ImageData = copy(image_data)

            # Fully filter the image data
            image_filter.fully_filter_image(temp_image_data)

            # Find the contours in the image
            temp_image_data.generate_contours_in_image()

            # Find the centroids of the contours in the image
            temp_image_data.calculate_image_centroids()

            # Visualize the result
            visualization.visualize_images(temp_image_data)

            # Update the confidence map with the new data acquired by the segmentations
            # TODO the confidence weights for each filter will need tuning
            if type(image_filter) is ImageFilterGrabCut:
                confidence_map = confidence_map + 0.375 * temp_image_data.expanded_image_mask
            if type(image_filter) is ImageFilterThreshold:
                confidence_map = confidence_map + 0.375 * temp_image_data.expanded_image_mask

        # Show the updated confidence map
        cv2.imshow("Confidence Map", np.uint8(confidence_map * 255))
        cv2.waitKey(1)

        # Overlay the confidence map on to the original image
        # TODO the confidence map should be overlaid onto the original image
        """cv2.imshow("Confidence Map Segmentation",
                   np.uint8(visualization.create_mask_overlay_array(temp_image_data.original_image,
                                                           confidence_map, 1)))"""
