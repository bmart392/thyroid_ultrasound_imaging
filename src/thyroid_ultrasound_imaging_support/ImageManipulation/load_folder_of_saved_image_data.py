"""
Contains the code for the load_folder_of_saved_image_data function.
"""

# Import standard python packages
from os import listdir
from fnmatch import fnmatch

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_support.Functions.generate_ordered_list_of_directory_contents import \
    generate_ordered_list_of_directory_contents


def load_folder_of_saved_image_data(source_folder_path: str) -> list:
    """
    Generates a list of ImageData objects from a folder of saved image data objects.

    Parameters
    ----------
    source_folder_path :
        the absolute path to the folder where the saved image data objects are located.

    Returns
    -------
    list
        A list of image data objects.
    """
    # Define a list to store the result
    list_of_image_data_objects = []

    # Capture the contents of the given folder
    folders_in_folder = list(generate_ordered_list_of_directory_contents(directory_path=source_folder_path,
                                                                    sort_indices=(0, 1),
                                                                    sort_by_string_allowed=True))[0]

    # For each folder,
    for full_path_to_sub_folder in folders_in_folder:

        # Create the full path to the sub-folder
        # full_path_to_sub_folder = source_folder_path + '/' + sub_folder_name

        # Get the contents of the sub-folder
        sub_folder_contents = listdir(full_path_to_sub_folder)

        # Check that the folder has 2 items in it and
        # that only one file has the .npz extension and
        # that only one file has the .txt extension
        if (len(sub_folder_contents) == 2 and
                (fnmatch(sub_folder_contents[0], '*.npz') != fnmatch(sub_folder_contents[1], '*.npz')) and
                (fnmatch(sub_folder_contents[0], '*.txt') != fnmatch(sub_folder_contents[1], '*.txt'))):

            # Create a new image data object and add it to the return list
            list_of_image_data_objects.append(ImageData(image_data_location=full_path_to_sub_folder))

    return list_of_image_data_objects
