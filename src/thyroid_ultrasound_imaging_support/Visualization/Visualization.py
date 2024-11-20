"""
Contains Visualization object and associated function.
"""

# Import standard python packages
from math import ceil
from matplotlib.pyplot import axes, figure, show
from numpy import array, zeros, ones, uint8, newaxis, where, repeat
from cv2 import GC_BGD, GC_PR_FGD, imshow, waitKey, line, \
    namedWindow, startWindowThread, setMouseCallback, destroyWindow

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageFilter.FilterConstants import COLOR_RGB
from thyroid_ultrasound_imaging_support.Visualization.VisualizationConstants import *
from thyroid_ultrasound_imaging_support.Visualization.create_mask_display_array import create_mask_display_array
from thyroid_ultrasound_imaging_support.Visualization.create_mask_overlay_array import create_mask_overlay_array, \
    COLORIZED, COLOR_BGR
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.Visualization.add_centroids_on_image import add_centroids_on_image
from thyroid_ultrasound_imaging_support.Visualization.add_cross_and_center_lines_on_image import \
    add_cross_and_center_lines_on_image


# TODO - Dream - Fill in missing visualization schemes


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
        self.image_centering_goal_offset = 0

        self.image_titles = {
            SHOW_ORIGINAL: TITLE_ORIGINAL,
            SHOW_CROPPED: TITLE_CROPPED,
            SHOW_RECOLOR: TITLE_RECOLOR,
            SHOW_BLUR: TITLE_PRE_PROCESSED,
            SHOW_MASK: TITLE_RESULT_MASK,
            SHOW_POST_PROCESSED_MASK: TITLE_POST_PROCESSED_MASK,
            SHOW_SURE_FOREGROUND: TITLE_SURE_FOREGROUND,
            SHOW_SURE_BACKGROUND: TITLE_SURE_BACKGROUND,
            SHOW_PROBABLE_FOREGROUND: TITLE_PROBABLE_FOREGROUND,
            SHOW_INITIALIZED_MASK: TITLE_INITIALIZED_MASK,
            SHOW_CENTROIDS_ONLY: TITLE_CENTROIDS_ONLY,
            SHOW_CENTROIDS_CROSS_ONLY: TITLE_CENTROIDS_CROSS_ONLY,
            SHOW_MASK_OVERLAY: TITLE_MASK_OVERLAY,
            SHOW_CENTROIDS_CROSS_OVERLAY: TITLE_CENTROIDS_CROSS_OVERLAY,
            SHOW_MASK_CENTROIDS_CROSS_OVERLAY: TITLE_MASK_CENTROIDS_CROSS_OVERLAY,
            SHOW_FOREGROUND: TITLE_FOREGROUND,
            SHOW_SKIN_APPROXIMATION: TITLE_BALANCE_LINE_APPROXIMATION,
            SHOW_GRABCUT_USER_INITIALIZATION_0: TITLE_GRABCUT_USER_INITIALIZATION_0,
        }

    def visualize_images(self, image_data: ImageData, specific_visualizations: list = None,
                         image_array: array = None, image_title: str = None, callback_function=None,
                         callback_inputs: list = None, skin_approximation_parameters: dict = None):
        """
            Visualize the data within an image_data object as a single image or part of an image stream.

            Parameters
            ----------
            skin_approximation_parameters :
            callback_inputs :
            callback_function :
            image_title :
            image_array :
            image_data: ImageData
                object containing the data to be visualized
            specific_visualizations
                list containing specific visualizations to show. If none are provided, shows visualizations found
                in self.visualizations.
        """
        if image_array is not None and image_title is not None and callback_function is not None and callback_inputs is not None:
            self.show_basic_image(image_array, image_title,
                                  callback_function=callback_function,
                                  callback_inputs=callback_inputs)
        else:
            if specific_visualizations is None:
                visualizations_to_show = self.visualizations
            else:
                visualizations_to_show = specific_visualizations

            # Calculate the number of visualizations
            num_visuals = len(visualizations_to_show)

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
            for visual in visualizations_to_show:

                # Provide default value in case one is not assigned
                image_to_show = None

                if visual == SHOW_ORIGINAL:
                    image_to_show = image_data.original_image

                elif visual == SHOW_CROPPED:
                    image_to_show = image_data.cropped_image

                elif visual == SHOW_RECOLOR:
                    image_to_show = image_data.colorized_image

                elif visual == SHOW_BLUR:
                    image_to_show = image_data.pre_processed_image

                elif visual == SHOW_MASK:
                    if image_data.image_mask is not None:
                        image_to_show = create_mask_display_array(image_data.image_mask)

                elif visual == SHOW_POST_PROCESSED_MASK:
                    if image_data.post_processed_mask is not None:
                        image_to_show = create_mask_display_array(image_data.post_processed_mask)

                elif visual == SHOW_FOREGROUND:
                    if image_data.original_image is not None and image_data.post_processed_mask is not None:
                        image_to_show = create_mask_overlay_array(image_data.down_sampled_image,
                                                                  image_data.post_processed_mask,
                                                                  base_image_color=COLOR_RGB,
                                                                  output_image_color=COLOR_BGR,
                                                                  overlay_method=COLORIZED, overlay_color=(35, 0, 0))

                elif visual == SHOW_SURE_FOREGROUND:
                    if image_data.sure_foreground_mask is not None:
                        image_to_show = create_mask_overlay_array(image_data.down_sampled_image,
                                                                  image_data.sure_foreground_mask,
                                                                  base_image_color=COLOR_RGB,
                                                                  output_image_color=COLOR_RGB,
                                                                  overlay_method=COLORIZED, overlay_color=(0, 35, 0))

                elif visual == SHOW_SURE_BACKGROUND:
                    if image_data.sure_background_mask is not None:
                        image_to_show = create_mask_overlay_array(image_data.down_sampled_image,
                                                                  image_data.sure_background_mask,
                                                                  base_image_color=COLOR_RGB,
                                                                  output_image_color=COLOR_BGR,
                                                                  overlay_method=COLORIZED, overlay_color=(0, 0, 35))

                elif visual == SHOW_PROBABLE_FOREGROUND:
                    if image_data.probable_foreground_mask is not None:
                        image_to_show = create_mask_overlay_array(image_data.down_sampled_image,
                                                                  image_data.probable_foreground_mask,
                                                                  base_image_color=COLOR_RGB,
                                                                  output_image_color=COLOR_BGR,
                                                                  overlay_method=COLORIZED)

                elif visual == SHOW_INITIALIZED_MASK:
                    if image_data.segmentation_initialization_mask is not None:
                        image_to_show = create_mask_display_array(image_data.segmentation_initialization_mask,
                                                                  multiplier=INITIALIZED_MASK_MULTIPLIER)

                    else:
                        image_to_show = create_mask_display_array(ones(image_data.image_mask.shape, uint8), 255)

                elif visual == SHOW_GRABCUT_USER_INITIALIZATION_0:
                    if image_data.segmentation_initialization_mask is not None:
                        initialization_mask = image_data.segmentation_initialization_mask[:, :, newaxis]
                        image_to_show = image_data.cropped_image * where((initialization_mask == GC_BGD) |
                                                                         (initialization_mask == GC_PR_FGD), 0, 1)

                elif visual == SHOW_CENTROIDS_ONLY:

                    if len(image_data.contour_centroids) > 0:
                        # Create an empty mask
                        image_to_show = zeros(image_data.down_sampled_image.shape, uint8)

                        # Draw each centroid from the image
                        image_to_show = add_centroids_on_image(image_to_show, image_data)

                elif visual == SHOW_CENTROIDS_CROSS_ONLY:

                    if len(image_data.contour_centroids) > 0:
                        # Create an empty mask
                        image_to_show = zeros(image_data.down_sampled_image.shape, uint8)

                        # Draw each centroid from the image and a cross
                        image_to_show = \
                            add_cross_and_center_lines_on_image(image_to_show, image_data,
                                                                goal_location=self.image_centering_goal_offset)

                elif visual == SHOW_MASK_OVERLAY:

                    # Copy the original image
                    image_to_show = image_data.original_image

                elif visual == SHOW_CENTROIDS_CROSS_OVERLAY:

                    # Copy the original image
                    image_to_show = image_data.original_image

                elif visual == SHOW_MASK_CENTROIDS_CROSS_OVERLAY:

                    # Copy the original image
                    image_to_show = image_data.original_image

                elif visual == SHOW_SKIN_APPROXIMATION:

                    # Copy the original image
                    image_to_show = image_data.original_image

                    image_to_show = line(image_to_show,
                                         (0, int(skin_approximation_parameters[BEST_FIT_SHARED_LINE_B])),
                                         (image_to_show.shape[1], int((skin_approximation_parameters[
                                                                           BEST_FIT_SHARED_LINE_A] *
                                                                       image_to_show.shape[1]) +
                                                                      skin_approximation_parameters[
                                                                          BEST_FIT_SHARED_LINE_B])),
                                         color=(255, 0, 0), thickness=2)

                else:
                    raise Exception("Visualization type not recognized.")

                # Add message image title to image title
                image_title = image_data.image_title + ":\n " + self.image_titles[visual]

                # Remove carriage return if continuous images are shown
                if self.image_mode == IMG_CONTINUOUS:
                    image_title = image_title.replace("\n", " ")

                # Show the image using the parameters determined above
                if image_to_show is not None:
                    if len(image_to_show) > 0:
                        self.show_basic_image(image_to_show, image_title,
                                              all_axes[current_axis_number])

                # Increment the current axis number for the next loop
                # current_axis_number = current_axis_number + 1

            # If displaying a single image, adjust the layout of the figure and show it
            if self.image_mode == IMG_SINGLE and visualization_fig is not None:
                visualization_fig.tight_layout()
                show()

    def show_basic_image(self, image_array: array, image_title: str, image_axes: axes = None,
                         x_axis_label: str = "X", y_axis_label: str = "Y",
                         callback_function=None,
                         callback_inputs: list = None):
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
        callback_function
            the function to call based when the given event occurs.
        callback_inputs
            the inputs to pass to the callback function.
        """

        if callback_function is not None and callback_inputs is not None:
            # Show the image
            startWindowThread()
            namedWindow(image_title)
            setMouseCallback(image_title, callback_function, callback_inputs)
            imshow(image_title, image_array)
            waitKey(0)
            destroyWindow(image_title)

        elif self.image_mode == IMG_SINGLE:

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
            startWindowThread()
            namedWindow(image_title)
            imshow(image_title, image_array)
            waitKey(1)

        # Raise an exception if an incorrect imaging mode is passed in.
        else:
            raise Exception("Image mode not recognized.")


def test_imshow():
    imshow("test", zeros((100, 100), uint8))
    waitKey(0)
    print("shown")
