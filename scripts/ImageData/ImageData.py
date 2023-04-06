"""
Contains code for ImageData class.
"""

# Import standard libraries
import cv2
import matplotlib.pyplot as plt
from numpy import sum, uint8
from os import listdir
from os.path import isdir, isfile


class ImageData:
    """
    A class for holding image data and its filtered components.
    """

    def __init__(self, image_data=None, image_filepath=None,
                 image_color=None, image_title=None,
                 image_size_x=0, image_size_y=0,
                 segmentation_initialization_mask=None):

        # Define basic parameters of the image
        self.image_color = image_color
        self.image_title = image_title

        # Save the mask used to initialize the segmentation of this image
        self.segmentation_initialization_mask = segmentation_initialization_mask

        # --------------------------------------------------------------
        # Save the data for the image according to how it has been given
        # --------------------------------------------------------------

        # If the image is only given as a filepath, read the image from the file
        if image_filepath is not None and image_data is None:
            self.original_image = cv2.imread(image_filepath)

        # Else if the image is given as an array, save the array
        elif image_data is not None:
            self.original_image = image_data

        # Otherwise initialize an empty object
        else:
            self.original_image = None

        # --------------------------------------------------------------

        # Calculate the size of the image if one is given
        if self.original_image is not None:
            self.image_size_x = self.original_image.shape[1]
            self.image_size_y = self.original_image.shape[0]

        # Otherwise set it to a default value
        else:
            self.image_size_x = image_size_x
            self.image_size_y = image_size_y

        # --------------------------------------------------------------
        # Create empty parameters for use by the filters
        # !!! Order of parameters represent order of operations expected
        # --------------------------------------------------------------
        self.cropped_image = None
        self.colorized_image = None
        self.pre_processed_image = None
        self.image_mask = None
        self.expanded_image_mask = None
        self.sure_foreground_mask = None
        self.sure_background_mask = None
        self.probable_foreground_mask = None

        # Create empty parameters for storing the centroids and contours
        self.contours_in_image = None
        self.contour_centroids = []

        self.image_figure = None

    def generate_contours_in_image(self):
        self.contours_in_image = []
        if self.expanded_image_mask is not None and not sum(self.expanded_image_mask) == 0:
            contours_in_image, hierarchy = cv2.findContours(
                self.expanded_image_mask.astype(uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_NONE
            )
            temp_contours = sorted(contours_in_image,
                                   key=cv2.contourArea,
                                   reverse=True)
            for contour in temp_contours:
                if cv2.contourArea(contour) > 10:
                    self.contours_in_image.append(contour)

    def calculate_image_centroids(self):

        # create empty array to return when no centroids are found
        result = []

        # calculate the centroid of each contour
        for contour in self.contours_in_image:
            temp_moments = cv2.moments(contour)
            temp_centroid_x = int(temp_moments["m10"] / temp_moments["m00"])
            temp_centroid_y = int(temp_moments["m01"] / temp_moments["m00"])
            self.contour_centroids.append((temp_centroid_x, temp_centroid_y))

        # append the centroid of the largest contour found
        result.append((self.contours_in_image[0], self.contour_centroids[0]))

        # if 2 or more contours were found and the second contour also big, include it
        if len(self.contour_centroids) >= 2:
            if (cv2.contourArea(self.contours_in_image[1]) >=
                    0.9 * cv2.contourArea(self.contours_in_image[0])):
                result.append((self.contours_in_image[1], self.contour_centroids[1]))

        return result

    def plot_images(self, figure=None):
        if figure is None and self.image_figure is None:
            figure_dimensions = (4, 12)
            figure = plt.figure(figsize=figure_dimensions)
        if self.image_figure is None:
            self.image_figure = figure

        num_image_rows = 5
        num_image_cols = 1
        row_index = 1

        for image in [self.original_image,
                      self.colorized_image,
                      self.pre_processed_image,
                      self.image_mask
                      ]:
            if image is not None:
                temp_image_axis = self.image_figure.add_subplot(
                    num_image_rows,
                    num_image_cols,
                    row_index
                )
                temp_image_axis.set_title("")
                temp_image_axis.set_xticks([])
                temp_image_axis.set_yticks([])
                plt.imshow(image, cmap='gray')
                row_index = row_index + 1
        self.image_figure.tight_layout()


def import_images_as_image_data_objects(file_path: str, color_scheme_of_images: int, image_number_offset: int):
    # Check if the file path to the images is valid.
    if not isdir(file_path):
        raise "File path is not valid."

    # Get a list of the files in the directory
    file_names = listdir(file_path)

    # Create an empty list in which to store the image data objects
    created_objects = list([None]) * (len(file_names))

    for file_name in file_names:

        # Append the file path to the file name.
        file_name_with_path = file_path + '/' + file_name

        # Check that the file name is valid.
        if not isfile(file_name_with_path):
            raise "File name is not valid."

        # Create a new image data object and add it to the list.
        image_position = int(file_name[file_name.find("_") + 1: file_name.find(".")]) - image_number_offset
        created_objects[image_position] = (
            ImageData(image_filepath=file_name_with_path, image_color=color_scheme_of_images,
                      image_title=file_name))

    # Return the list of objects.
    return created_objects
