# Import standard packages
from math import ceil, floor
from random import random
import numpy as np
import matplotlib.pyplot as plt
import cv2
from panda3d.core import Triangulator
from copy import copy

# Import ImageData object
from scripts.ImageData.ImageData import ImageData

# Import custom constants
from scripts.Visualizations.VisualizationConstants import *

# Import array helper functions
from scripts.ArrayHelpers.ArrayHelpers import *

# Import ImageFilterGrabCut object
from scripts.Filters.ImageFilterGrabCut import ImageFilterGrabCut


class Visualization:

    def __init__(self, image_mode: int, visualizations: list, visualization_title: str = None,
                 num_cols_in_fig: int = 2):
        self.image_mode = image_mode
        self.visualizations = visualizations
        self.visualization_title = visualization_title
        self.num_cols_in_fig = num_cols_in_fig

    def visualize_images(self, image_data: ImageData):
        """
            Visualize the data within an image_data object as a single image or part of an image stream.

            Parameters
            ----------
            image_data: ImageData
                object containing the data to be visualized
        """

        # Calculate the number of visualizations
        num_visuals = len(self.visualizations)

        # Create figure and plots required for showing a single image
        if self.image_mode == IMG_SINGLE:

            # Calculate the number of rows needed to make that many plots
            num_rows_in_fig = ceil(num_visuals / self.num_cols_in_fig)

            # Create an empty figure to hold the visualizations
            visualization_fig = plt.figure(dpi=300)

            # Add title to overall figure.
            if self.visualization_title is not None:
                visualization_fig.suptitle(self.visualization_title)

            # Create an empty list to hold the axes needed for each visual
            all_axes = [None] * num_visuals

            # Add subplots for each visual
            for ii in range(num_visuals):
                all_axes[ii] = visualization_fig.add_subplot(num_rows_in_fig, self.num_cols_in_fig, 1 + ii)

        else:

            # Create a None list of the correct length
            all_axes = [None] * num_visuals

        # Create a variable to store which axis is being used for the visualization
        current_axis_number = 0

        for visual in self.visualizations:
            if visual == SHOW_ORIGINAL:
                self.show_basic_image(image_data.original_image, "Original Image",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_CROPPED:
                self.show_basic_image(image_data.cropped_image, "Cropped Image",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_RECOLOR:
                self.show_basic_image(image_data.colorized_image, "Recolored Image",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_BLUR:
                self.show_basic_image(image_data.pre_processed_image, "Blurred Image",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_MASK:
                self.show_basic_image(self.create_mask_display_array(image_data.image_mask),
                                      "Image Mask\nfrom Segmentation",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_EXPANDED_MASK:
                self.show_basic_image(self.create_mask_display_array(image_data.expanded_image_mask),
                                      "Full Size Image Mask\nfrom Segmentation",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_FOREGROUND:
                self.show_basic_image(self.create_mask_overlay_array(image_data.original_image,
                                                                     image_data.expanded_image_mask),
                                      "Foreground of the Image",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_SURE_FOREGROUND:
                self.show_basic_image(self.create_mask_overlay_array(image_data.original_image,
                                                                     image_data.sure_foreground_mask),
                                      "Sure Foreground of Image",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_SURE_BACKGROUND:
                self.show_basic_image(self.create_mask_overlay_array(image_data.original_image,
                                                                     image_data.sure_background_mask),
                                      "Sure Background of Image",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_PROBABLE_FOREGROUND:
                self.show_basic_image(self.create_mask_overlay_array(image_data.original_image,
                                                                     image_data.probable_foreground_mask),
                                      "Probable Foreground of Image",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_INITIALIZED_MASK:
                if image_data.segmentation_initialization_mask is not None:
                    self.show_basic_image(self.create_mask_display_array(image_data.segmentation_initialization_mask,
                                                                         multiplier=INITIALIZED_MASK_MULTIPLIER),
                                          "Mask for Initializing\nSegmentation",
                                          self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_GRABCUT_USER_INITIALIZATION_0:
                initialization_mask = image_data.segmentation_initialization_mask[:, :, np.newaxis]
                self.show_basic_image(image_data.cropped_image *
                                      np.where((initialization_mask == cv2.GC_BGD) |
                                               (initialization_mask == cv2.GC_PR_BGD), 0, 1),
                                      "Initialized Region of Image",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_CENTROIDS_ONLY:

                # Create an empty mask
                temp_image_array = np.zeros(image_data.original_image.shape, np.uint8)

                # Draw each centroid from the image
                for centroid in image_data.contour_centroids:
                    temp_image_array = draw_circle(centroid, 10, 127, temp_image_array)

                self.show_basic_image(temp_image_array, "Image Centroids.",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_CENTROIDS_CROSS_ONLY:

                # Create an empty mask
                temp_image_array = np.zeros(image_data.original_image.shape, np.uint8)

                # Define line width
                line_width = 4

                # Draw each centroid from the image
                for centroid in image_data.contour_centroids:
                    temp_image_array = draw_circle(centroid, 10, 127, temp_image_array)

                # Draw vertical line
                temp_image_array = draw_rectangle("corner",
                                                  (floor((image_data.image_size_x / 2) - ceil(line_width / 2)), 0),
                                                  line_width, image_data.image_size_y, FULL_VALUE_RGB_MULTIPLIER,
                                                  temp_image_array)

                # Draw horizontal line
                temp_image_array = draw_rectangle("corner",
                                                  (0, floor((image_data.image_size_y / 2) - ceil(line_width / 2))),
                                                  image_data.image_size_x, line_width, FULL_VALUE_RGB_MULTIPLIER,
                                                  temp_image_array)

                self.show_basic_image(temp_image_array, "Image Centroids with Crosshair.",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_MASK_OVERLAY:

                # Copy the original image
                temp_image_array = image_data.original_image

                # Define line width
                line_width = 4

                # Draw each centroid from the image
                for centroid in image_data.contour_centroids:
                    temp_image_array = draw_circle(centroid, 10, 127, temp_image_array)

                # Draw vertical line
                temp_image_array = draw_rectangle("corner",
                                                  (floor((image_data.image_size_x / 2) - ceil(line_width / 2)), 0),
                                                  line_width, image_data.image_size_y, FULL_VALUE_RGB_MULTIPLIER,
                                                  temp_image_array)

                # Draw horizontal line
                temp_image_array = draw_rectangle("corner",
                                                  (0, floor((image_data.image_size_y / 2) - ceil(line_width / 2))),
                                                  image_data.image_size_x, line_width, FULL_VALUE_RGB_MULTIPLIER,
                                                  temp_image_array)

                self.show_basic_image(temp_image_array, "Mask Overlaid\non Original Image.",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_CENTROIDS_CROSS_OVERLAY:

                # Copy the original image
                temp_image_array = image_data.original_image

                self.show_basic_image(temp_image_array, "Centroids Overlaid\non Original Image.",
                                      self.image_mode, all_axes[current_axis_number])
            elif visual == SHOW_MASK_CENTROIDS_CROSS_OVERLAY:

                # Copy the original image
                temp_image_array = image_data.original_image

                self.show_basic_image(temp_image_array, "Mask, Centroids, and Cross\nOverlaid on Original Image.",
                                      self.image_mode, all_axes[current_axis_number])
            else:
                raise Exception("Visualization type not recognized.")

            # Increment the current axis number for the next loop
            current_axis_number = current_axis_number + 1

        if self.image_mode == IMG_SINGLE:
            visualization_fig.tight_layout()
            plt.show()

    @staticmethod
    def show_basic_image(image_array: np.array, image_title: str, image_mode: int, image_axes: plt.axes,
                         visualization_title: str = None, x_axis_label: str = "X", y_axis_label: str = "Y"):

        if image_mode == IMG_SINGLE:

            if len(image_array.shape) == 2:
                image_array = np.repeat(image_array[:, :, np.newaxis], 3, axis=2)

            # Plot the image array
            image_axes.imshow(image_array)

            # Add labels and title
            image_axes.set_title(image_title, size=6)
            image_axes.set_xlabel(x_axis_label, fontsize=6)
            image_axes.set_ylabel(y_axis_label, fontsize=6)
            image_axes.tick_params(axis='both', which='major', labelsize=4)

        elif image_mode == IMG_CONTINUOUS:

            # Prepend the visualization title on to the image title if necessary.
            if visualization_title is not None:
                image_title = visualization_title + " - " + image_title

            cv2.imshow(image_title, image_array)
            cv2.waitKey(1)

        else:
            raise Exception("Image mode not recognized.")

    @staticmethod
    def create_mask_overlay_array(base_image: np.array, mask: np.array, fade_rate: float = 0.5):
        return base_image * mask[:, :, np.newaxis] + np.uint8(
            base_image * (1 - mask)[:, :, np.newaxis] * fade_rate
        )

    @staticmethod
    def create_mask_display_array(mask: np.array, multiplier: int = 255):
        return np.uint8(mask * multiplier)


def user_input_crop_coordinates(image_data: ImageData, crop_coordinates: list = None, ):
    """
    Prompt the user to select coordinates to crop the image.
    """

    # Do nothing if the user chooses not to crop the image.
    if crop_coordinates is None:

        # Define window name
        window_name = "Image Crop Selection"

        # Ask the user if they would like to crop the image.
        # crop_input = input("Would you like to crop the image? (y/n)\n")

        # Define characteristics of the shapes to show on the image
        text_color = (128, 0, 128)
        point_color = (0, 0, 255)
        point_radius = 3
        line_color = (0, 255, 0)
        line_thickness = 1
        box_color = (0, 255, 0)
        box_thickness = 1

        # Create a copy of the original image to prevent damaging the original object
        temp_image_array = copy(image_data.original_image)

        # Add text to the image informing the user
        temp_image_array = display_text_on_image(temp_image_array,
                                                 "Select the upper left-hand crop corner." +
                                                 " Press any key to continue.")

        # Display the image for the user
        cv2.imshow(window_name, temp_image_array)

        # Define a variable to store the result of the user's click
        first_corner = [int(0), int(0)]

        # Ask user to select the first corner to use to crop the image
        cv2.setMouseCallback(window_name, click_event, first_corner)
        cv2.waitKey(0)

        # Recopy the original image to clear the previous text
        temp_image_array = copy(image_data.original_image)

        # Display a circle at the point selected to assist the user
        temp_image_array = cv2.circle(temp_image_array, first_corner, point_radius,
                                      point_color, -1)

        # Display two lines to show how the image has been cropped so far
        temp_image_array = cv2.line(temp_image_array, (first_corner[0], first_corner[1]),
                                    (first_corner[0], temp_image_array.shape[0]),
                                    line_color, line_thickness)
        temp_image_array = cv2.line(temp_image_array, (first_corner[0], first_corner[1]),
                                    (temp_image_array.shape[1], first_corner[1]),
                                    line_color, line_thickness)

        # Display text to assist the user
        temp_image_array = display_text_on_image(temp_image_array,
                                                 "Select the lower right-hand crop corner." +
                                                 " Press any key to continue.")
        # Show the new image to the user
        cv2.imshow(window_name, temp_image_array)

        # Define a variable to store the result of the user's click
        second_corner = [int(image_data.original_image.shape[1] - 1), int(image_data.original_image.shape[0] - 1)]

        # Ask the user to select the corner to use to crop the image
        cv2.setMouseCallback(window_name, click_event, second_corner)
        cv2.waitKey(0)

        # Ensure that the second corner was chosen properly
        if not (second_corner[0] > first_corner[0] and second_corner[1] > first_corner[1]):
            raise Exception("Second corner was not selected below and to the right of the first.")

        # Recopy the original image to clear the previous annotations
        temp_image_array = copy(image_data.original_image)

        # Draw a rectangle to show how the image will be cropped
        temp_image_array = cv2.rectangle(temp_image_array, first_corner, second_corner, box_color, box_thickness)

        # Display text to assist the user
        temp_image_array = display_text_on_image(temp_image_array,
                                                 "Preview of crop region." +
                                                 " Press any key to continue.")

        # Show the new image to the user
        cv2.imshow(window_name, temp_image_array)
        cv2.waitKey(0)

        # Destroy all windows when finished
        cv2.destroyAllWindows()

        # Display the points selected for the user
        print("1st Corner: (" + str(first_corner[0]) + ", " + str(first_corner[1]) + ")\n" +
              "2nd Corner: (" + str(second_corner[0]) + ", " + str(second_corner[1]) + ")")

        # Return the points selected
        return [first_corner, second_corner]

    else:
        # Return the points passed in
        return crop_coordinates


def user_input_polygon_points(image_data: ImageData, polygon_use: str, list_of_points: list = None):
    # Do nothing if the background points have been passed in
    if list_of_points is None:
        # Define a window name to display
        window_name = 'Image Background Selection'

        # Define colors to use for displaying information
        text_color = (128, 0, 128)
        point_color = (0, 0, 255)

        # Create a copy of the image array so that the original is not changed
        temp_image_array = copy(image_data.cropped_image)

        # Add text to the image to inform the user what needs to happen.
        temp_image_array = display_text_on_image(temp_image_array,
                                                 'Select points to define the border',
                                                 text_origin=(5, 10), text_size=0.375)
        temp_image_array = display_text_on_image(temp_image_array,
                                                 'of the ' + polygon_use + ".",
                                                 text_origin=(5, 20), text_size=0.375)
        temp_image_array = display_text_on_image(temp_image_array,
                                                 'Press any key to continue.',
                                                 text_origin=(5, 30), text_size=0.375)
        # Show the image in the properly named window
        cv2.imshow(window_name, temp_image_array)

        # Initialize the additional inputs needed for the mouseclick callback function
        list_of_points = []
        list_of_line_colors = []
        callback_inputs = [temp_image_array, point_color, window_name, list_of_line_colors, list_of_points]

        # Set the callback function
        cv2.setMouseCallback(window_name, boundary_click_event, callback_inputs)

        # Wait for the user to close the window
        cv2.waitKey(0)

        # Ensure that at least three points have been selected
        if len(list_of_points) < 3:
            raise Exception("Not enough points were selected to make a polygon." +
                            " At least 3 points must be selected.")

        """list_of_triangles = create_convex_triangles_from_points(list_of_points)

        temp_image_array = copy(image_data.cropped_image)
        temp_image_array = display_text_on_image(temp_image_array, "Preview of " + polygon_use + " triangles.",
                                                 text_origin=(5, 10), text_size=0.375)
        for triangle in list_of_triangles:
            this_image_array = copy(temp_image_array)
            this_image_array = cv2.polylines(this_image_array, [np.array(triangle).reshape((-1, 1, 2))], True,
                                             (0, 0, 255), 2)
            cv2.imshow(window_name, this_image_array)
            cv2.waitKey(0)"""

        # Destroy the window when the input is complete
        cv2.destroyWindow(window_name)

        # Return the list of points generated by the user
        print(list_of_points)
        return list_of_points

    else:
        # Return points passed into the function
        return list_of_points


def boundary_click_event(event: int, x, y, flags, additional_inputs: list):
    """
    Mouse-click callback function to capture a mouse click, draw a dot, and connect previously selected points
    together with a randomly colored line.
    """

    # Only do something when the left mouse button has been clicked
    if event == cv2.EVENT_LBUTTONDOWN:

        # Create a new copy of the image to annotate
        temp_image_array = copy(additional_inputs[0])

        # Rename the individual additional_inputs for clarity
        list_of_points = additional_inputs[-1]
        list_of_colors = additional_inputs[-2]
        point_color = additional_inputs[1]
        window_name = additional_inputs[2]

        # Define attributes of the drawn elements
        point_radius = 3
        intermediate_line_width = 1
        final_line_color = (255, 255, 255)
        final_line_width = 2

        # Add the newly selected point to the list of existing points
        list_of_points.append((x, y))

        # Iterate through the list of points to show them on the image
        for point in list_of_points:
            temp_image_array = cv2.circle(temp_image_array, point, point_radius, point_color, -1)

        # Connect the points together with lines
        # There must be at least 2 points for this to occur
        if len(list_of_points) > 1:

            # Create a new color to use for the line and save it for future use
            list_of_colors.append(generate_random_color())

            # Iterate through the list of points to draw each line using the appropriate color
            for i in range(1, len(list_of_points)):
                temp_image_array = cv2.line(temp_image_array, list_of_points[i - 1], list_of_points[i],
                                            list_of_colors[i - 1], intermediate_line_width)

        # Connect the first and last point with a different line
        # There must be at least three points selected for this to occur
        if len(list_of_points) > 2:
            temp_image_array = cv2.line(temp_image_array, list_of_points[0], list_of_points[-1],
                                        final_line_color, final_line_width)

        # Show the updated image
        cv2.imshow(window_name, temp_image_array)


def click_event(event: int, x, y, flags, additional_inputs: list):
    """
    Return the position selected on the click of a mouse.
    """

    # Do nothing until the left mouse button is clicked
    if event == cv2.EVENT_LBUTTONDOWN:
        # Save the click location
        additional_inputs[0] = int(x)
        additional_inputs[1] = int(y)


def display_text_on_image(image: np.array, text: str, text_origin: tuple = (15, 35), text_color: tuple = (128, 0, 128),
                          text_size: float = 0.5):
    """
    Use the cv2 built-in method to add text to an image.
    """
    return cv2.putText(image, text, text_origin, cv2.FONT_HERSHEY_SIMPLEX, text_size, text_color, thickness=1,
                       bottomLeftOrigin=False)


def generate_random_color(num_channels: int = 3, color_value_type: type = np.uint8):
    """
    Generate a random color.
    """

    # Select the maximum color value based on the color value type
    # Raise an exception if the color value type is not recognized
    if color_value_type == np.uint8:
        max_value = 255
    elif color_value_type == float:
        max_value = 1
    else:
        raise Exception("Image value type not recognized. Only float or uint8 accepted.")

    # Create a color based on the number of channels in the image and the maximum color value selected above
    # Raise an exception if the number of channels in the image is not recognized
    if num_channels == 3:
        color = [1, 1, 1]
        for i in range(len(color)):
            color[i] = round(color[i] * random() * max_value)
    elif num_channels == 1:
        color = random() * max_value
    else:
        raise Exception('Number of channels not recognized. Only 1 or 3 channel images accepted.')

    # Return a tuple as the color
    return tuple(color)


def create_convex_triangles_from_points(list_of_points: list):
    # Create a triangulator object
    triangulator = Triangulator()

    # Iterate through each point in the list
    for point in list_of_points:
        # Add each point to the pool of vertices and then to the current polygon
        triangulator.addPolygonVertex(triangulator.addVertex(x=point[0], y=point[1]))

    # Create the triangles of the polygons
    triangulator.triangulate()

    # Define list to store polygons
    list_of_triangles = []

    # Add the indices for each triangle to the list of triangle indices
    for i in range(triangulator.getNumTriangles()):
        list_of_triangles.append([triangulator.getTriangleV0(i),
                                  triangulator.getTriangleV1(i),
                                  triangulator.getTriangleV2(i)])
        for j in range(len(list_of_triangles[-1])):
            vertex = triangulator.getVertex(list_of_triangles[-1][j])
            list_of_triangles[-1][j] = [int(vertex.x), int(vertex.y)]

    # Replace the triangle indices with the points themselves
    return list_of_triangles


if __name__ == "__main__":
    test_list_of_points = [[0, 0], [1, 0], [1, 1], [0, 1]]

    create_convex_triangles_from_points(test_list_of_points)

    print("done")
