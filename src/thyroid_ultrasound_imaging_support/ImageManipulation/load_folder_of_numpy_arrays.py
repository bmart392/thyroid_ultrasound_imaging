"""
Defines the load_folder_of_numpy_arrays function.
"""

# Import standard packages
from os.path import isdir, isfile
from os import listdir
from cv2 import imshow, waitKey
from matplotlib.pyplot import pause
from numpy import load, uint8

# Import custom python packages
from thyroid_ultrasound_imaging_support.Visualization.display_process_timer import *


def load_folder_of_numpy_arrays(folder_path: str,
                                starting_index: int = None,
                                ending_index: int = None,
                                return_index_numbers: bool = False,
                                convert_results_to_lists: bool = False):
    """
    Generate a list of numpy arrays from numpy arrays saved in the given folder. Numpy arrays MUST be named in the form
    of <WORD>_####.npy, such as 'Slice_00001.npy'.

    Parameters
    ----------
    folder_path
        The folder location where the arrays are stored.
    starting_index
        The index at which to start converting the arrays.
    ending_index
        The index at which to stop converting the arrays.
    return_index_numbers
        If True, returns the identifying index number of each array generated.
    convert_results_to_lists
        If True, the result will be a list of lists rather than a list of arrays
    """

    # Check if the folder path to the arrays is valid.
    if not isdir(folder_path):
        raise "Folder path is not valid."

    # Get a list of the files in the directory
    file_names = listdir(folder_path)

    # Create a dictionary to store the loaded arrays
    array_dictionary = {}

    # Create a list to store the result of the function in
    created_objects = []

    # Ensure that the starting and ending indices are valid
    if starting_index is None:
        starting_index = 0
    elif starting_index < 0:
        raise Exception("Invalid starting index of " + str(starting_index) + " given.")
    if ending_index is None:
        ending_index = 10 ** 10
    elif ending_index < starting_index:
        raise Exception("Ending index of " + str(ending_index) + " cannot be bigger than the starting index of"
                        + str(starting_index) + ".")

    # Iterate through the list of file names found
    for file_name in file_names:

        # Only select numpy arrays
        if 'npy' in file_name:

            # Append the file path to the file name.
            file_name_with_path = folder_path + '/' + file_name

            # Check that the file name is valid.
            if not isfile(file_name_with_path):
                raise "File name is not valid."

            # Extract the array identifier from the image name
            array_number = int(int(file_name[file_name.find("_") + 1: file_name.find(".")]))

            # Only add arrays that are within the bounds selected
            if starting_index <= array_number <= ending_index:
                array_dictionary[array_number] = load(file_name_with_path)

    # Store a sorted list of keys used in the dictionary
    sorted_keys = sorted(array_dictionary.keys())

    # Add the images to the resulting list in order
    if convert_results_to_lists:
        for key in sorted_keys:
            created_objects.append(array_dictionary[key].tolist())
    else:
        for key in sorted_keys:
            created_objects.append(array_dictionary[key])

    # Return the correct objects based on the user selection
    if return_index_numbers:
        return created_objects, sorted_keys
    else:
        return created_objects


if __name__ == '__main__':

    # Choose whether the arrays should be visualized
    visualize_arrays = False

    # Define the folder location and image offset
    test_path = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/Test/Experimentation/' \
                'Experiment_2024-01-12/GroundTruths'
    test_starting_index = None

    # Note the current time
    start_of_process_time = time()

    # Create the list of arrays
    temp_result, array_names = load_folder_of_numpy_arrays(test_path, test_starting_index,
                                                           return_index_numbers=True)

    # Display how long it took to generate the list and how many arrays were generated
    display_process_timer(start_of_process_time, "Array Generation from Folder")
    print("Number of Arrays Generated: " + str(len(array_names)))

    # Display the arrays to ensure that they were properly imported
    if visualize_arrays:
        for temp_array in temp_result:
            # Display the image
            imshow("Read from Folder Test", temp_array*uint8(255))

            # Wait to ensure the image remains on screen
            waitKey(1)

            # Wait 0.05 seconds between showing images
            pause(0.05)

    # End the test
    print('Done')
