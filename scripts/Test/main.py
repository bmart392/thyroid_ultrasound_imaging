import numpy as np

from scripts.ImageData.ImageData import ImageData, import_images_as_image_data_objects
from scripts.Filters.ImageFilterThreshold import ImageFilterThreshold
from scripts.Filters.ImageFilterGrabCut import ImageFilterGrabCut
from scripts.Visualizations.Visualization import Visualization, user_input_polygon_points, user_input_crop_coordinates, \
    create_convex_triangles_from_points
from scripts.Filters.FilterHelper import COLOR_BGR
from scripts.Visualizations.VisualizationConstants import *
from scripts.ArrayHelpers.create_mask_array import create_mask_array, ShapeTypes, RectangleTypes
from scripts.ArrayHelpers.ArrayHelpers import create_mask_array_from_triangles, get_average_value_from_triangles
from scripts.Filters.FilterHelper import crop_image
import cv2
from copy import copy
import matplotlib.pyplot as plt

if __name__ == '__main__':
    input = True  # True = define input, false = use values

    series = True  # False = series 2, True = series 1

    image_type = IMG_SINGLE

    # Read in images as array of images
    if series:
        images = import_images_as_image_data_objects('Images/Series1', COLOR_BGR, 30)  # Read images from Series 1
    else:
        images = import_images_as_image_data_objects('Images/Series2', COLOR_BGR, 20)  # Read images from Series 2

    # Define list of visualizations to show
    list_of_visualizations = [
        [
            # SHOW_ORIGINAL,
            # SHOW_CROPPED,
            # SHOW_BLUR,
            # SHOW_INITIALIZED_MASK,
            # SHOW_GRABCUT_USER_INITIALIZATION_0,
            # SHOW_EXPANDED_MASK,
            SHOW_FOREGROUND,
            # SHOW_SURE_FOREGROUND,
            # SHOW_SURE_BACKGROUND,
            # SHOW_PROBABLE_FOREGROUND
            # SHOW_CENTROIDS_CROSS_ONLY,
            # SHOW_MASK_OVERLAY
        ],
        [
            # SHOW_ORIGINAL,
            # SHOW_CROPPED,
            # SHOW_BLUR,
            # SHOW_MASK,
            # SHOW_INITIALIZED_MASK,
            # SHOW_GRABCUT_USER_INITIALIZATION_0,
            # SHOW_EXPANDED_MASK,
            SHOW_FOREGROUND,
            # SHOW_SURE_FOREGROUND,
            # SHOW_SURE_BACKGROUND,
            # SHOW_PROBABLE_FOREGROUND
            # SHOW_CENTROIDS_CROSS_ONLY,
            # SHOW_MASK_OVERLAY
        ]
    ]

    # Create the visualization objects
    visualizations = [Visualization(image_type, list_of_visualizations[0],
                                    visualization_title="Grabcut Filter", num_cols_in_fig=3),
                      Visualization(image_type, list_of_visualizations[1],
                                    visualization_title="Threshold Filter", num_cols_in_fig=3),
                      ]

    # Save a test image
    # test_image = ImageData(image_filepath='Images/Series2/Slice_30.png', image_color=COLOR_BGR)  # For Testing
    test_image = images[0]  # For Running

    # Use GUI to find the crop coordinates for the image -> then save coordinates
    if input:
        image_crop_coordinates = user_input_crop_coordinates(test_image)
    else:
        if series:
            image_crop_coordinates = [[328, 67], [540, 578]]  # Series 1 - Slice 30
        else:
            image_crop_coordinates = [[198, 197], [538, 494]]  # Series 2 - Slice 30

    # Crop the test image
    test_image = crop_image(True, image_crop_coordinates, test_image)

    # Select the background points -> then save the coordinates
    #
    if input:
        list_of_points_for_background_polygon = user_input_polygon_points(test_image, "background")
    else:
        if series:
            list_of_points_for_background_polygon = [(21, 52), (183, 58), (173, 252), (27, 236)]  # Series 1 - Slice 30
        else:
            list_of_points_for_background_polygon = [(2, 2), (334, 6), (321, 131), (163, 134), (49, 142),
                                             (23, 179)]  # Series 1 - Slice 30

    # Select the foreground points -> then save the coordinates
    if input:
        list_of_points_for_foreground_polygon = user_input_polygon_points(test_image, "foreground")
    else:
        if series:
            list_of_points_for_foreground_polygon = [(102, 292), (157, 318), (101, 345)]  # Series 1 - Slice 30
        else:
            list_of_points_for_foreground_polygon = [(27, 212), (128, 157), (58, 259)]  # Series 1 - Slice 30

    # Convert the points to triangles
    list_of_background_triangles = create_convex_triangles_from_points(list_of_points_for_background_polygon)
    list_of_foreground_triangles = create_convex_triangles_from_points(list_of_points_for_foreground_polygon)

    # Use the polygons to generate the initial mask
    initial_mask = create_mask_array_from_triangles(list_of_background_triangles, list_of_foreground_triangles,
                                                    test_image.cropped_image.shape[:2])

    """list_of_points_for_thresholding = user_input_polygon_points(test_image, "Threshold Values")
    list_of_polygons_for_thresholding = create_convex_triangles_from_points(list_of_points_for_thresholding)
    thresholding_parameters = get_average_value_from_triangles(list_of_polygons_for_thresholding, test_image.original_image)
"""
    if series:
        thresholding_parameters = (100, 120)
    else:
        thresholding_parameters = (80, 110)

    # Create the image filters
    image_filters = [ImageFilterGrabCut(initial_mask,
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

    # Run the algorithm
    # images = [test_image]

    """user_created_mask_shapes = [
        # (ShapeTypes.RECTANGLE, RectangleTypes.FLAT_CORNER, 170, 110, 300 - 170, 260 - 110, None, 'red', cv2.GC_BGD),
        # (ShapeTypes.CIRCLE, None, 240, 150, None, None, 70, 'red', cv2.GC_BGD),
        (ShapeTypes.CIRCLE, None, 110, 325, None, None, 50, 'green', cv2.GC_FGD),
        (ShapeTypes.RECTANGLE, RectangleTypes.FLAT_CORNER, 0, 0, 200 - 0, 225 - 0, None, 'red', cv2.GC_BGD),
        (ShapeTypes.RECTANGLE, RectangleTypes.FLAT_CORNER, 0, 420, 200 - 0, 490 - 420, None, 'red', cv2.GC_BGD),
        (ShapeTypes.RECTANGLE, RectangleTypes.FLAT_CORNER, 0, 0, 25 - 0, 490 - 0, None, 'red', cv2.GC_BGD),
        # (ShapeTypes.RECTANGLE, RectangleTypes.FLAT_CORNER, 70, 120, 120 - 70, 180 - 120, None, 'green', cv2.GC_FGD),
        # (ShapeTypes.RECTANGLE, RectangleTypes.FLAT_CORNER, 335, 110, 385 - 335, 175 - 110, None, 'green', cv2.GC_FGD)
    ]

    image_crop_coordinates = [[330, 75], [530, 575]]
    # image_crop_coordinates = [[150, 200], [550, 480]]

    user_created_mask = create_mask_array(user_created_mask_shapes,
                                          (image_crop_coordinates[1][1] - image_crop_coordinates[0][1],
                                           image_crop_coordinates[1][0] - image_crop_coordinates[0][0]))"""

    # a = input("Press Enter to Run.")

    confidence_map = np.zeros(test_image.original_image.shape[:2])

    # Test with the first image
    # for image_data in [ImageData(image_filepath='Images/Series1/Slice_30.png', image_color=COLOR_BGR)]:
    # for image_data in [ImageData(image_filepath='Images/Series2/Slice_20.png', image_color=COLOR_BGR)]:
    for image_data in images:

        confidence_map = confidence_map / 4

        for image_filter, visualization in zip(image_filters, visualizations):
            # Create a temporary copy of the image data
            temp_image_data: ImageData = copy(image_data)

            print(temp_image_data.image_title)

            # Fully filter the image data
            image_filter.fully_filter_image(temp_image_data)

            # Find the contours in the image
            temp_image_data.generate_contours_in_image()

            # Find the centroids of the contours in the image
            temp_image_data.calculate_image_centroids()

            # Visualize the result
            visualization.visualize_images(temp_image_data)

            if type(image_filter) is ImageFilterGrabCut:
                image_filter.use_previous_image_mask = True

            # a = input("Press Enter to Continue.")

            if type(image_filter) is ImageFilterGrabCut:
                confidence_map = confidence_map + 0.375 * temp_image_data.expanded_image_mask
                print(confidence_map.max)
            if image_filter is ImageFilterThreshold:
                confidence_map = confidence_map + 0.375 * temp_image_data.expanded_image_mask

        cv2.imshow("Confidence Map", np.uint8(confidence_map * 255))
        cv2.waitKey(1)

        """cv2.imshow("Confidence Map Segmentation",
                   np.uint8(visualization.create_mask_overlay_array(temp_image_data.original_image,
                                                           confidence_map, 1)))"""
    # Use this function to wait for user input
