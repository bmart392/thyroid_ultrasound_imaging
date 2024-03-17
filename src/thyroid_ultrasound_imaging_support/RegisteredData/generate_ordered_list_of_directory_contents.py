
# Import standard python packages
from operator import itemgetter
from os import listdir
from os.path import exists


def generate_ordered_list_of_directory_contents(directory_path: str,
                                                sort_indices: tuple) -> list:
    """
    Generates an ordered list of the contents of a given directory.
    The function assumes that all members of the directory use '_' to separate key parameters of the name.

    Parameters
    ----------
    directory_path :
        The directory in which to generate a list of contents.
    sort_indices :
        A tuple of indices representing which portion of the file name to sort by.
        Can be integer values between 1 and 5.

    Returns
    -------
    list
        A list of file
    """
    # Ensure that the directory exists
    if not exists(directory_path):
        raise Exception(directory_path + " is not a valid location.")

    # Ensure that the directory path ends with a back-slash
    if directory_path[-1] != '/':
        directory_path = directory_path + '/'

    # Define a destination for the sorted contents of the directory
    sorted_directory_contents = []

    # For each item in the directory
    for item_name in listdir(directory_path):

        # Generate the name of the item without any extensions
        try:
            item_name_without_file_extension = item_name[:item_name.index('.')]
        except ValueError:
            item_name_without_file_extension = item_name

        try:
            # Split the item name by underscores
            item_name_components = item_name_without_file_extension.split('_')

            # Define a temporary variable to store the individual components that will be sorted by
            temp_components = []

            # Add the individual relevant components to the entry for the item
            for index in sort_indices:
                temp_components.append(item_name_components[index])

            # Add the item name at the end
            temp_components.append(item_name)

            # Add the temporary list to the sorted list
            sorted_directory_contents.append(temp_components)
        except IndexError:
            print(item_name_without_file_extension + ' is not named correctly to be sorted.')

    # Create an itemgetter with the appropriate number of sort indices
    if len(sort_indices) == 1:
        temp_itemgetter = itemgetter(sort_indices[0])
    elif len(sort_indices) == 2:
        temp_itemgetter = itemgetter(sort_indices[0],
                                     sort_indices[1])
    elif len(sort_indices) == 3:
        temp_itemgetter = itemgetter(sort_indices[0],
                                     sort_indices[1],
                                     sort_indices[2])
    elif len(sort_indices) == 4:
        temp_itemgetter = itemgetter(sort_indices[0],
                                     sort_indices[1],
                                     sort_indices[2],
                                     sort_indices[3])
    elif len(sort_indices) == 5:
        temp_itemgetter = itemgetter(sort_indices[0],
                                     sort_indices[1],
                                     sort_indices[2],
                                     sort_indices[3],
                                     sort_indices[4])
    else:
        raise Exception("The number of indices to sort by, " + str(len(sort_indices)) +
                        ", is not supported by the function.")

    # Sort the contents
    sorted_directory_contents.sort(key=temp_itemgetter)

    # Define a list to store just the file names in order
    temp_sorted_list = []

    # Add the files names to the temporary list
    for item in sorted_directory_contents:
        temp_sorted_list.append(directory_path + item[-1])

    # Return the temporary list
    return temp_sorted_list


if __name__ == '__main__':
    file_path = '/home/ben/thyroid_ultrasound_data/testing_and_validation/saved_image_data_objects'

    print(generate_ordered_list_of_directory_contents(file_path, (0, 1)))
