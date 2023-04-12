"""
Contains Visualization object and associated function.
"""
# Import standard packages
from math import ceil
from matplotlib.pyplot import axes, figure, show
from numpy import array, zeros, ones, uint8, newaxis, where, repeat
from cv2 import GC_BGD, GC_PR_BGD, imshow, waitKey, line, circle

# Import custom constants
from scripts.d_Visualizations.VisualizationConstants import *

# Import custom objects
from scripts.b_ImageData.ImageData import ImageData

# TODO Fill in missing visualization schemes


class Visualization:
    """
    A class to define a visualization object.
    """

    def __init__(self, image_mode: int, visualizations: list, visualization_title: str = None,
                 num_cols_in_fig: int = 2):
        """
        Create a Visualization object.

        Parameters
        ----------
        image_mode
            a selector for the imaging mode being shown. Options are single or continuous.
        visualizations
            a list of integers specifying which visualizations will be created.
            Acceptable options are contained in the VisualizationConstants file.
        visualization_title
            the message to display as the title of the visualization.
        num_cols_in_fig
            an integer determining how many columns of visualizations will be shown when visualizing a single image.
        """
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
            visualization_fig = figure(dpi=300)

            # Add title to overall figure.
            if self.visualization_title is not None:
                visualization_fig.suptitle(self.visualization_title)

            # Create an empty list to hold the axes needed for each visual
            all_axes = [None] * num_visuals

            # Add subplots for each visual
            for ii in range(num_visuals):
                all_axes[ii] = visualization_fig.add_subplot(num_rows_in_fig, self.num_cols_in_fig, 1 + ii)

        else:

            # Define the visualization figure as None
            visualization_fig = None

            # Create a None list of the correct length
            all_axes = [None] * num_visuals

        # Create a variable to store which axis is being used for the visualization
        current_axis_number = 0

        # Iterate through each visualization type
        # Define the image to display and the image title
        for visual in self.visualizations:
            if visual == SHOW_ORIGINAL:
                image_to_show = image_data.original_image
                image_title = "Original Image"

            elif visual == SHOW_CROPPED:
                image_to_show = image_data.cropped_image
                image_title = "Cropped Image"

            elif visual == SHOW_RECOLOR:
                image_to_show = image_data.colorized_image
                image_title = "Recolored Image"

            elif visual == SHOW_BLUR:
                image_to_show = image_data.pre_processed_image
                image_title = "Blurred Image"

            elif visual == SHOW_MASK:
                image_to_show = self.create_mask_display_array(image_data.image_mask)
                image_title = "Image Mask\nfrom Segmentation"

            elif visual == SHOW_EXPANDED_MASK:
                image_to_show = self.create_mask_display_array(image_data.expanded_image_mask)
                image_title = "Full Size Image Mask\nfrom Segmentation"

            elif visual == SHOW_FOREGROUND:
                image_to_show = self.create_mask_overlay_array(image_data.original_image,
                                                               image_data.expanded_image_mask)
                image_title = "Foreground of the Image"

            elif visual == SHOW_SURE_FOREGROUND:
                image_to_show = self.create_mask_overlay_array(image_data.original_image,
                                                               image_data.sure_foreground_mask),
                image_title = "Sure Foreground of Image"

            elif visual == SHOW_SURE_BACKGROUND:
                image_to_show = self.create_mask_overlay_array(image_data.original_image,
                                                               image_data.sure_background_mask)
                image_title = "Sure Background of Image"

            elif visual == SHOW_PROBABLE_FOREGROUND:
                image_to_show = self.create_mask_overlay_array(image_data.original_image,
                                                               image_data.probable_foreground_mask)
                image_title = "Probable Foreground of Image"

            elif visual == SHOW_INITIALIZED_MASK:
                if image_data.segmentation_initialization_mask is not None:
                    image_to_show = self.create_mask_display_array(image_data.segmentation_initialization_mask,
                                                                   multiplier=INITIALIZED_MASK_MULTIPLIER)
                    image_title = "Mask for Initializing\nSegmentation"

                else:
                    image_to_show = self.create_mask_display_array(ones(image_data.image_mask.shape, uint8), 255)
                    image_title = "No Initialization Mask\nProvided"

            elif visual == SHOW_GRABCUT_USER_INITIALIZATION_0:
                initialization_mask = image_data.segmentation_initialization_mask[:, :, newaxis]
                image_to_show = image_data.cropped_image * where((initialization_mask == GC_BGD) |
                                                                 (initialization_mask == GC_PR_BGD), 0, 1)
                image_title = "Initialized Region of Image"

            elif visual == SHOW_CENTROIDS_ONLY:

                # Create an empty mask
                image_to_show = zeros(image_data.original_image.shape, uint8)

                # Draw each centroid from the image
                image_to_show = self.add_centroids_on_image(image_to_show, image_data)

                image_title = "Image Centroids"

            elif visual == SHOW_CENTROIDS_CROSS_ONLY:

                # Create an empty mask
                image_to_show = zeros(image_data.original_image.shape, uint8)

                # Draw each centroid from the image and a cross
                image_to_show = self.add_cross_and_centroids_on_image(image_to_show, image_data)

                image_title = "Image Centroids with Crosshair."

            elif visual == SHOW_MASK_OVERLAY:

                # Copy the original image
                image_to_show = image_data.original_image

                image_title = "Mask Overlaid\non Original Image."

            elif visual == SHOW_CENTROIDS_CROSS_OVERLAY:

                # Copy the original image
                image_to_show = image_data.original_image

                image_title = "Centroids Overlaid\non Original Image."

            elif visual == SHOW_MASK_CENTROIDS_CROSS_OVERLAY:

                # Copy the original image
                image_to_show = image_data.original_image

                image_title = "Mask, Centroids, and Cross\nOverlaid on Original Image."

            else:
                raise Exception("Visualization type not recognized.")

            # Remove carriage return if continuous images are shown
            if self.image_mode == IMG_CONTINUOUS:
                image_title = image_title.replace("\n", " ")

            # Show the image using the parameters determined above
            self.show_basic_image(image_to_show, image_title,
                                  all_axes[current_axis_number])

            # Increment the current axis number for the next loop
            current_axis_number = current_axis_number + 1

        # If displaying a single image, adjust the layout of the figure and show it
        if self.image_mode == IMG_SINGLE and visualization_fig is not None:
            visualization_fig.tight_layout()
            show()

    def show_basic_image(self, image_array: array, image_title: str, image_axes: axes,
                         x_axis_label: str = "X", y_axis_label: str = "Y"):
        """
        Display a given image array using standard options.

        Parameters
        ----------
        image_array
            a numpy array containing the image to be visualized.
        image_title
            the title describing the image.
        image_axes
            for single images displayed using matplotlib, this is the axis that the image
            will be displayed on.
        x_axis_label
            a label to display for the x-axis.
        y_axis_label
            a label to display for the y-axis.
        """

        if self.image_mode == IMG_SINGLE:

            if len(image_array.shape) == 2:
                image_array = repeat(image_array[:, :, newaxis], 3, axis=2)

            # Plot the image array
            image_axes.imshow(image_array)

            # Add labels and title
            image_axes.set_title(image_title, size=6)
            image_axes.set_xlabel(x_axis_label, fontsize=6)
            image_axes.set_ylabel(y_axis_label, fontsize=6)
            image_axes.tick_params(axis='both', which='major', labelsize=4)

        elif self.image_mode == IMG_CONTINUOUS:

            # Prepend the visualization title on to the image title if necessary.
            if self.visualization_title is not None:
                image_title = self.visualization_title + " - " + image_title

            # Show the image
            imshow(image_title, image_array)
            waitKey(1)

        # Raise an exception if an incorrect imaging mode is passed in.
        else:
            raise Exception("Image mode not recognized.")

    @staticmethod
    def create_mask_overlay_array(base_image: array, mask: array, fade_rate: float = 0.5):
        """
        Returns an array that overlays a binary mask over an image array.

        Parameters
        ----------
        base_image
            a numpy array containing the base image.
        mask
            a numpy array representing the mask to be overlaid.
        fade_rate
            the fade ratio to apply to the inverse of the mask.
            A smaller number darkens the remainder of the image.
        """
        return base_image * mask[:, :, newaxis] + uint8(
            base_image * (1 - mask)[:, :, newaxis] * fade_rate
        )

    @staticmethod
    def add_cross_to_image(image_to_show: array, vertical_line_color: tuple = (255, 255, 255),
                           horizontal_line_color: tuple = (255, 255, 255), line_thickness: int = 1):
        """
        Add two lines to a given image to form a cross centered on the image. Returns the input image array.

        Parameters
        ----------
        image_to_show
            a numpy array representing the image to show.
        vertical_line_color
            the color of the vertical line.
        horizontal_line_color
            the color of the horizontal line.
        line_thickness
            the thickness of the lines drawn.
        """

        # Draw vertical line
        image_to_show = line(image_to_show,
                             (int(image_to_show.shape[1] / 2), 0),
                             (int(image_to_show.shape[1] / 2), image_to_show.shape[0]),
                             color=vertical_line_color, thickness=line_thickness)

        # Draw horizontal line
        return line(image_to_show,
                    (0, int(image_to_show.shape[0] / 2)),
                    (image_to_show.shape[1], int(image_to_show.shape[0] / 2),),
                    color=horizontal_line_color, thickness=line_thickness)

    @staticmethod
    def add_centroids_on_image(image_to_show: array, image_data: ImageData,
                               dot_radius: int = 6, dot_color: tuple = (255, 0, 0)):
        """
        Draw a dot for each centroid from a given ImageData object on a given image. Returns the input image array.

        Parameters
        ----------
        image_to_show
            a numpy array representing the image to show.
        image_data
            the ImageData object that contains the centroids to show.
        dot_radius
            the size of the dot.
        dot_color
            the color of each dot.
        """
        # Draw each centroid from the image
        for centroid in image_data.contour_centroids:
            image_to_show = circle(image_to_show,
                                   centroid,
                                   radius=dot_radius,
                                   color=dot_color,
                                   thickness=-1)
        return image_to_show

    def add_cross_and_centroids_on_image(self, image_to_show: array, image_data: ImageData):
        """
        Add both a cross and the centroids from a given ImageData object to a given image array.
        This function draws the centroids, then the cross. Returns the input image array.

        Parameters
        ----------
        image_to_show
            a numpy array representing the image to show.
        image_data
            the ImageData object that contains the centroids to show.
        """
        return self.add_cross_to_image(self.add_centroids_on_image(image_to_show, image_data))

    @staticmethod
    def create_mask_display_array(mask: array, multiplier: int = 255):
        """
        Modifies a mask array so that it can be properly displayed.

        Parameters
        ----------
        mask
            a numpy array of the mask to display
        multiplier
            the value that all values in the mask will be multiplied by.
        """
        return uint8(mask * multiplier)
