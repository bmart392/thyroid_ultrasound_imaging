"""
Defines the generate_list_of_image_arrays function.
"""

# Import standard packages
from os.path import isdir, isfile
from os import listdir
from cv2 import imread, imshow, waitKey
from matplotlib.pyplot import pause

# Import custom python packages
from thyroid_ultrasound_imaging_support.Visualization.display_process_timer import *


def generate_list_of_image_arrays(folder_path: str,
                                  starting_index: int,
                                  ending_index: int = None) -> list:
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
    """

    # Check if the folder path to the images is valid.
    if not isdir(folder_path):
        raise "Folder path is not valid."

    # Get a list of the files in the directory
    file_names = listdir(folder_path)

    # Calculate the largest allowed index
    max_index = len(file_names)

    # Calculate the ending offset
    if ending_index is None:
        ending_index = starting_index + max_index - 1
    elif ending_index > max_index:
        ending_index = max_index

    # Create an empty list in which to store the image data objects
    created_objects: list = list([None]) * (ending_index - starting_index + 1)

    # Iterate through the list of file names found to ensure that
    # the image data objects are added to the list in sequential order.
    for file_name in file_names:

        # Append the file path to the file name.
        file_name_with_path = folder_path + '/' + file_name

        # Check that the file name is valid.
        if not isfile(file_name_with_path):
            raise "File name is not valid."

        # Calculate the position in the list to store the image
        image_position = int(int(file_name[file_name.find("_") + 1: file_name.find(".")]) - starting_index)

        # Place the read-in file at the correct position
        created_objects[image_position] = imread(file_name_with_path)

    # Ensure stream_images references the global variable
    return created_objects


if __name__ == '__main__':

    # Choose whether the images should be visualized
    visualize_images = False

    # Define the folder location and image offset
    test_path = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/Test/Images/2023-11-29_19-14'
    test_starting_index = 1

    # Note the current time
    start_of_process_time = time()

    # Create the list of images
    temp_result = generate_list_of_image_arrays(test_path, test_starting_index)

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
