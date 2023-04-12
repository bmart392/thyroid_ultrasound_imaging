"""
Contains code for ImageData class and helper function for creating image data objects from a folder of images.
"""

# Import standard libraries
import cv2
from numpy import sum, uint8, array
from os import listdir
from os.path import isdir, isfile


class ImageData:
    """
    A class for holding image data and its filtered components.
    """

    def __init__(self, image_data: array = None,
                 image_filepath: str = None,
                 image_color: int = None,
                 image_title: str = None,
                 image_size_x: int = 0,
                 image_size_y: int = int(0),
                 segmentation_initialization_mask: array = None):
        """
        Creates a new ImageData object.

        Parameters
        ----------
        image_data
            an array representing the image to use to create the ImageData object.
        image_filepath
            a string containing the path to an image file to use to create the ImageData object.
        image_color
            a constant representing the color scheme of the image.
        image_title
            a title for the image.
        image_size_x
            the size of the image in the x-axis.
        image_size_y
            the size of the image in the y-axis.
        segmentation_initialization_mask
            the mask used to initialize the segmentation of the image stored in this object.
            This is not used in all types of image filters.
        """

        # Check that an image has been given for object creation
        if image_data is None and image_filepath is None:
            raise Exception("An ImageData object could not be created because an image was not provided.")

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

    def generate_contours_in_image(self):
        """
        Generate contours from the 2-dimensional binary image stored in the expanded_image_mask field. Stores the
        result in the contours_in_image field of the current object.
        """

        # Define an empty list to store the result
        self.contours_in_image = []

        # Only create the contours if the expanded image mask has been created
        # AND
        # the expanded image mask contains any contours.
        if self.expanded_image_mask is not None and not sum(self.expanded_image_mask) == 0:

            # Use a built-in function to generate the contours
            contours_in_image, hierarchy = cv2.findContours(
                self.expanded_image_mask.astype(uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_NONE
            )

            # Sort the resulting contours by area with the largest contour first
            temp_contours = sorted(contours_in_image,
                                   key=cv2.contourArea,
                                   reverse=True)

            # Define a minimum area requirement
            minimum_contour_area = 10

            # Only add contours to the final list that exceed the minimum contour area requirement
            for contour in temp_contours:
                if cv2.contourArea(contour) > minimum_contour_area:
                    self.contours_in_image.append(contour)

    def calculate_image_centroids(self):
        """
        Calculate the centroids of each contour in the list of contours. Returns the result as a
        list of (x, y) coordinates.
        """

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


def import_images_as_image_data_objects(file_path: str, color_scheme_of_images: int, image_number_offset: int = None):
    """
    Import all image files found in given folder as ImageData objects. These images must be stored using the
    naming convention <WORD>_<NUMBER>.<EXT>

    Parameters
    ----------
    file_path
        the path to the folder where the image files are stored.
    color_scheme_of_images
        an integer representing the color scheme used in the image files.
    image_number_offset
        an integer equivalent to the number found in the name of the first image.
    """
    # Check if the file path to the images is valid.
    if not isdir(file_path):
        raise "File path is not valid."

    # Set the offset number to 0 if none is provided
    if image_number_offset is None:
        image_number_offset = 0

    # Get a list of the files in the directory
    file_names = listdir(file_path)

    # Create an empty list in which to store the image data objects
    created_objects: list = list([None]) * (len(file_names))

    # Iterate through the list of file names found to ensure that
    # the image data objects are added to the list in sequential order.
    for file_name in file_names:

        # Append the file path to the file name.
        file_name_with_path = file_path + '/' + file_name

        # Check that the file name is valid.
        if not isfile(file_name_with_path):
            raise "File name is not valid."

        # Create a new image data object and add it to the list.
        image_position = int(int(file_name[file_name.find("_") + 1: file_name.find(".")]) - image_number_offset)
        created_objects[image_position] = (
            ImageData(image_filepath=file_name_with_path, image_color=color_scheme_of_images,
                      image_title=file_name))

    # Return the list of objects.
    return created_objects
