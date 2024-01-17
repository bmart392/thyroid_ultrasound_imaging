"""
Defines the load_folder_of_image_files function.
"""

# Import standard packages
from os.path import isdir, isfile
from os import listdir
from cv2 import imread, imshow, waitKey
from matplotlib.pyplot import pause

# Import custom python packages
from thyroid_ultrasound_imaging_support.Visualization.display_process_timer import *


def load_folder_of_image_files(folder_path: str,
                               starting_index: int = None,
                               ending_index: int = None,
                               return_image_numbers: bool = False):
    """
    Generate a list of image arrays from images saved in the given folder. Images MUST be named in the form of
    <WORD>_####.<FILE EXTENSION>, such as 'Slice_00001.png'.

    Parameters
    ----------
    folder_path
        The folder location where the images are stored.
    starting_index
        The index at which to start saving images.
    ending_index
        The index at which to stop saving images.
    return_image_numbers
        If True, returns the identifying index number of each image generated.
    """

    # Check if the folder path to the images is valid.
    if not isdir(folder_path):
        raise "Folder path is not valid."

    # Get a list of the files in the directory
    file_names = listdir(folder_path)

    # Create a dictionary to store the resulting images
    image_dictionary = {}

    # Create a list to store the result of the function in
    created_objects = []

    # Ensure that the starting and ending indices are valid
    if starting_index is None:
        starting_index = 0
    elif starting_index < 0:
        raise Exception("Invalid starting index of " + str(starting_index) + " given.")
    if ending_index is None:
        ending_index = 10**10
    elif ending_index < starting_index:
        raise Exception("Ending index of " + str(ending_index) + " cannot be bigger than the starting index of"
                        + str(starting_index) + ".")

    # Iterate through the list of file names found to ensure that
    # the image data objects are added to the list in sequential order.
    for file_name in file_names:

        # Append the file path to the file name.
        file_name_with_path = folder_path + '/' + file_name

        # Check that the file name is valid.
        if not isfile(file_name_with_path):
            raise "File name is not valid."

        # Calculate the position in the list to store the image
        image_number = int(int(file_name[file_name.find("_") + 1: file_name.find(".")]))

        # Only add images that are within the bounds selected
        if starting_index <= image_number <= ending_index:
            image_dictionary[image_number] = imread(file_name_with_path)

    # Store a sorted list of keys used in the dictionary
    sorted_keys = sorted(image_dictionary.keys())

    # Add the images to the resulting list in order
    for key in sorted_keys:
        created_objects.append(image_dictionary[key])

    # Return the correct objects based on the user selection
    if return_image_numbers:
        return created_objects, sorted_keys
    else:
        return created_objects


if __name__ == '__main__':

    # Choose whether the images should be visualized
    visualize_images = False

    # Define the folder location and image offset
    test_path = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/Test/Images/2023-11-29_19-14'
    test_starting_index = None

    # Note the current time
    start_of_process_time = time()

    # Create the list of images
    temp_result, image_names = load_folder_of_image_files(test_path, test_starting_index,
                                                          return_image_numbers=True)

    # Display how long it took to generate the list and how many images were generated
    display_process_timer(start_of_process_time, "Image-array Generation from Folder")
    print("Number of Images Generated: " + str(len(listdir(test_path))))

    # Display the images to ensure that they were properly imported
    if visualize_images:
        for image in temp_result:

            # Display the image
            imshow("Read from Folder Test", image)

            # Wait to ensure the image remains on screen
            waitKey(1)

            # Wait 0.05 seconds between showing images
            pause(0.05)

    # End the test
    print('Done')
