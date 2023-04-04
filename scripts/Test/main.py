from scripts.ImageData.ImageData import ImageData, import_images_as_image_data_objects
from scripts.Filters.ImageFilterThreshold import ImageFilterThreshold
from scripts.Filters.ImageFilterGrabCut import ImageFilterGrabCut
from scripts.Visualizations.Visualization import Visualization
from scripts.Filters.FilterHelper import COLOR_BGR
from scripts.Visualizations.VisualizationConstants import *
from scripts.ArrayHelpers.create_mask_array import create_mask_array, ShapeTypes, RectangleTypes
import cv2
from copy import copy
import matplotlib.pyplot as plt


def create_initialization_mask():
    pass


# TODO - Modify main file to run from a stream of images
# TODO -    Create function to read all images from a directory into an array of ImageData objects - DONE

if __name__ == '__main__':
    # Read in images as array of images
    images = import_images_as_image_data_objects('Images/Series1', COLOR_BGR, 30)

    # Define list of visualizations to show
    list_of_visualizations = [
        # SHOW_ORIGINAL,
        # SHOW_CROPPED,
        SHOW_BLUR,
        SHOW_INITIALIZED_MASK,
        # SHOW_GRABCUT_USER_INITIALIZATION_0,
        # SHOW_EXPANDED_MASK,
        SHOW_FOREGROUND,
        # SHOW_SURE_FOREGROUND,
        # SHOW_SURE_BACKGROUND,
        # SHOW_PROBABLE_FOREGROUND
        # SHOW_CENTROIDS_CROSS_ONLY,
        # SHOW_MASK_OVERLAY
    ]

    # Create the visualization objects

    # Save a test image

    # Use GUI to find the crop coordinates for the image -> then save coordinates

    # Crop the test image

    # Select the background points -> then save the coordinates

    # Select the foreground points -> then save the coordinates

    # Convert the points to polygons

    # Use the polygons to generate the initial mask

    # Create the image filters

    # Run the algorithm

    user_created_mask_shapes = [
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
                                           image_crop_coordinates[1][0] - image_crop_coordinates[0][0]))

    # Create list of filters to test
    image_filters = [ImageFilterGrabCut(user_created_mask,
                                        analysis_mode=False,
                                        image_crop_included=True,
                                        image_crop_coordinates=image_crop_coordinates
                                        ),
                     # ImageFilterThreshold(analysis_mode=True,
                     #                     image_crop_included=True,
                     #                     image_crop_coordinates=image_crop_coordinates),
                     ]

    # Create visualization object
    visualizations = [Visualization(IMG_CONTINUOUS, list_of_visualizations,
                                    visualization_title="Grabcut Filter", num_cols_in_fig=3),
                      # Visualization(IMG_SINGLE, list_of_visualizations,
                      #              visualization_title="Threshold Filter", num_cols_in_fig=3),
                      ]

    image_crop_coordinates = visualizations[0].user_input_crop_coordinates(
        ImageData(image_filepath='Images/Series1/Slice_30.png', image_color=COLOR_BGR),
        image_filters[0])

    list_of_points_for_background_polygon = visualizations[0].user_input_polygon_points(
        ImageData(image_filepath='Images/Series1/Slice_30.png', image_color=COLOR_BGR),
        image_filters[0]
    )

    # Test with the first image
    # for image_data in [ImageData(image_filepath='Images/Series1/Slice_30.png', image_color=COLOR_BGR)]:
    # for image_data in [ImageData(image_filepath='Images/Series2/Slice_20.png', image_color=COLOR_BGR)]:
    for image_data in images:

        for image_filter, visualization in zip(image_filters, visualizations):
            # Create a temporary copy of the image data
            temp_image_data = copy(image_data)

            print(temp_image_data.image_title)

            # Fully filter the image data
            image_filter.fully_filter_image(temp_image_data)

            # Find the contours in the image
            temp_image_data.generate_contours_in_image()

            # Find the centroids of the contours in the image
            temp_image_data.calculate_image_centroids()

            # Visualize the result
            visualization.visualize_images(temp_image_data)

            image_filter.use_previous_image_mask = True

            # a = input("Press Enter to Continue.")

    # Use this function to wait for user input
