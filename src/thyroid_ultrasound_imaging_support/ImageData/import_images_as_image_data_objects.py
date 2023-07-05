"""
Contains code for import_images_as_data_objects function.
"""

# Import standard libraries
from os import listdir
from os.path import isdir, isfile

# Import custom functions and constants
from ImageData import ImageData


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
