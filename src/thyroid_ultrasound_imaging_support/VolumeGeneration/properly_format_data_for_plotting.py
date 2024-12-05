"""
File containing the properly_format_data_for_plotting function definition.
"""


# Import standard python packages
from pandas import DataFrame
from numpy import array, ndarray

# Import custom python packages


# Define the default headings to use for the input_data
DEFAULT_X_HEADING: str = 'X (m)'
DEFAULT_Y_HEADING: str = 'Y (m)'
DEFAULT_Z_HEADING: str = 'Z (m)'
DEFAULT_FULL_HEADING: tuple = (DEFAULT_X_HEADING, DEFAULT_Y_HEADING, DEFAULT_Z_HEADING)
POINT_SIZE: str = 'Point Size'
POINT_COLOR: str = 'Point Color'


def properly_format_data_for_plotting(input_data) -> (ndarray, ndarray, ndarray):
    """
    Converts a collection of point data into a panda DataFrame with standard headings.

    Parameters
    ----------
    input_data : ndarray or list or tuple
        The object containing the data to convert

    Returns
    -------
    Returns :
        A tuple containing the x, y, and z data in that order.
    """

    # If given a numpy array,
    if type(input_data) == ndarray:
        return properly_format_data_from_array(input_data)  # Create a DataFrame from it

    # If given a list or tuple,
    elif type(input_data) == (list or tuple):
        # If the list is formatted as a collection of points or
        # if the list is formatted as collections of individual x, y, and z values,
        if len(input_data) > 0 and len(input_data[0]) == 3 or len(input_data) == 3 and len(input_data[0]) > 0:
            # Convert the list to an array and create a data frame from it
            return properly_format_data_from_array(array(input_data))
        else:
            raise ValueError('The list or tuple is not formatted in an acceptable format.')
    else:
        raise TypeError('Data of type ' + str(type(input_data)) + ' cannot be used for this function.')


def properly_format_data_from_array(array_data: ndarray) -> (ndarray, ndarray, ndarray):
    """
    Converts a numpy array containing point data into three properly formatted arrays.

    Parameters
    ----------
    array_data :
        A numpy array containing data either as a collection of points or a collection of x, y, and z values.

    Returns
    -------
    Returns :
        A tuple containing the x, y, and z data in that order.

    """

    # If the data is not formatted correctly as 3 collections of x, y, and z data,
    if len(array_data.shape) == 1 and array_data.shape[0] == 3:
        array_data = array([array_data]).transpose()  # Reshape it
    # If the input data is not properly formatted as 3 columns and n rows,
    elif array_data.shape[1] == 3 and array_data.shape[0] != 3:
        array_data = array_data.transpose()  # Transpose the rows and columns
    elif array_data.shape[0] != 3:
        raise ValueError('Array input_data has shape ' + str(array_data.shape) +
                         ' which cannot be transformed to have 3 columns.')

    # Return each part of the data as a single array
    return array_data[0], array_data[1], array_data[2]


if __name__ == '__main__':
    for temp_data in [array([[0], [0], [0]]), [[0, 0, 0], [1, 1, 1]], [[0, 1], [0, 1], [0, 1]]]:
        print(convert_data_to_pandas_dataframe(temp_data))
