"""
Contains the code for the load_folder_of_saved_registered_data function.
"""
# Import standard python packages
from os.path import exists
from os import listdir

# Import custom python packages
from thyroid_ultrasound_imaging_support.RegisteredData.RegisteredData import RegisteredData, \
    REGISTERED_DATA_FOLDER_NAME, NUM_CONTENTS_IN_REGISTERED_DATA_FOLDER
from thyroid_ultrasound_imaging_support.RegisteredData.generate_ordered_list_of_directory_contents import \
    generate_ordered_list_of_directory_contents


def load_folder_of_saved_registered_data(directory_path: str) -> list:
    """
    Generates a list of RegisteredData objects from a directory where those objects have been saved.

    Parameters
    ----------
    directory_path :
        The path to where the registered data objects are stored.

    Returns
    -------
    list
        A list of RegisteredData objects.
    """
    # Ensure the path is valid before continuing
    if not exists(directory_path):
        raise Exception(directory_path + " is not a valid path.")

    # Define the list where the generated objects will be stored
    loaded_registered_data = []

    # For each item in the directory
    for registered_data_item in generate_ordered_list_of_directory_contents(
            directory_path=directory_path,
            sort_indices=(1, 2)):

        # Ensure that registered data is stored in the item
        if REGISTERED_DATA_FOLDER_NAME in registered_data_item and \
                len(listdir(registered_data_item)) == NUM_CONTENTS_IN_REGISTERED_DATA_FOLDER:

            try:
                # Add a new RegisteredData object to the list of found objects
                loaded_registered_data.append(RegisteredData(source_file_path=registered_data_item))
            except Exception:
                pass

    return list(loaded_registered_data)
