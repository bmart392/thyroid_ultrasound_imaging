"""
Contains code for creating single line representations of data.
"""

# Import standard python packages
from numpy import array, product, uint8
from rospy import Time
from ast import literal_eval

# Define a placeholder value for none values
NONE_PLACEHOLDER: str = "NONE"

# Define the delimiter for array values
ARRAY_DELIMITER: str = ","

# Define the delimiter character
DELIMITER: str = ":"

# Define the end of line character
END_OF_LINE: str = ""

# Define values for the different data types
INT: str = "int"
UINT8: str = "uint8"
FLOAT: str = "float"
STRING: str = "str"

# Define values for the different types of single lines
SIMPLE: str = "simple"
ARRAY: str = "array"
TIME: str = "time"


def create_single_line_simple_data(field_name: str, data, previous_string: str) -> str:
    """
    Creates a single line string representation of a string, integer, float, or None object.
    The single line is added on to the previous string and separated by a carriage return.
    The single line string is always formatted as "<field_name>:<data>\n".
    A placeholder value is inserted into the string whenever a None type is given.
    """

    # Convert data to a string if it is an int
    if data is int:
        data = str(data)
        data_type = INT

    # Convert the data to a string if it is an int
    elif data is float:
        data = str(data)
        data_type = FLOAT

    # Convert the data to a string if it is None
    elif data is None:
        data = NONE_PLACEHOLDER
        data_type = NONE_PLACEHOLDER

    else:

        # If the data is not a string, raise an exception
        if data is not str:
            raise Exception("The class of the data field is not a string, it is a " + str(type(data)))

        # Otherwise set the data type as string
        data_type = str

    # Otherwise return the string form
    return (previous_string + SIMPLE + DELIMITER + field_name + DELIMITER + data_type +
            DELIMITER + data + END_OF_LINE + "\n")


def create_single_line_array_data(field_name: str, data: array, previous_string: str) -> str:
    """
    Creates a single line string representation of a numpy array or None object.
    The single line is added on to the previous string and separated by a carriage return.
    The single line string is always formatted as "<field_name>:<data.shape>:<data>\n".
    A placeholder value is inserted into the string whenever a None type is given.
    """

    # If the data is None
    if data is None:

        # Replace it with the placeholder value
        string_data = NONE_PLACEHOLDER
        data_shape = NONE_PLACEHOLDER
        data_type = NONE_PLACEHOLDER

    else:

        # Ensure the data is an array before continuing
        if data is not array:
            # Raise an exception to the user if it is not an array
            raise Exception("The class of the data field is not an array, it is a " + str(type(data)))

        # Determine the data type of the data within the array
        if data.dtype == int:
            data_type = INT
        elif data.dtype == uint8:
            data_type = UINT8
        elif data.dtype == float:
            data_type = FLOAT

        # Raise an exception if the data type is not recognized
        else:
            raise Exception("The type of the data field " + str(data.dtype) + " was not recognized.")

        # Define a variable for the string form of the data
        string_data = ""

        # Add each data value to the string version
        for value in data.reshape(product(data.shape)):
            string_data = string_data + str(value) + ARRAY_DELIMITER

        # Remove the extra comma from the end of the string
        string_data = string_data[:-1]

        # Determine the shape of the original data
        data_shape = str(data.shape)

    # Return the string form of the data
    return (previous_string + ARRAY + DELIMITER + field_name + DELIMITER + data_type + DELIMITER + data_shape +
            DELIMITER + string_data + END_OF_LINE + "\n")


def create_single_line_time_stamp(field_name: str, data: Time, previous_string: str) -> str:
    """
    Creates a single line string representation of a time stamp.
    The single line is added on to the previous string and separated by a carriage return.
    The single line string is always formatted as "<field_name>:<data.secs>:<data.nsecs>\n".
    """

    # If the data field is not a time object
    if data is not Time:
        # Raise an exception to the user
        raise Exception("The class of the data field is not Time, it is " + str(type(data)))

    # Return the string form of the data
    return (previous_string + TIME + DELIMITER + field_name + DELIMITER + str(data.secs) +
            DELIMITER + str(data.nsecs) + END_OF_LINE + "\n")


def rebuild_data(single_line: str):
    """
    Rebuilds data from a single line of text. The single line must start with the type of data contained within.
    """

    # Split up the data based on the known data delimiter
    split_data = single_line.split(DELIMITER)

    # Pull out the type of data stored in the single line
    single_line_type = split_data[0]

    # Rebuild the data using the appropriate function
    if single_line_type == SIMPLE and len(split_data) == 4:
        return rebuild_simple_data(split_data[1:])

    elif single_line_type == ARRAY and len(split_data) == 5:
        return rebuild_array_data(split_data[1:])

    elif single_line_type == TIME and len(split_data) == 4:
        return rebuild_time(split_data[1:])

    # Otherwise raise and exception if data type is not recognized
    else:
        raise Exception("Type " + single_line_type + " with " + str(len(split_data)) + " fields was not recognized.")


def rebuild_simple_data(split_data: list):
    # Pull out the data type and the data from the string
    data_type = split_data[1]
    data = split_data[2]

    # Remove the carriage return if it is included in the data
    data = data.replace("\n", "")

    # Convert the data depending on the type
    if data_type == STRING:
        pass
    elif data_type == INT:
        data = int(data)
    elif data_type == FLOAT:
        data = float(data)
    elif data_type == NONE_PLACEHOLDER:
        data = None

    # Otherwise raise an exception if the data type is not recognized
    else:
        raise Exception("The data type of '" + data_type + "' was not recognized.")

    # Return the field name and the data
    return split_data[0], data


def rebuild_array_data(split_data: list):
    # Pull out the data type, data shape, the data from the string
    data_type = split_data[1]
    data_shape = split_data[2]
    data = split_data[3]

    # Remove the carriage return if it is included in the data
    data = data.replace("\n", "")

    # Define a temporary list to store the result
    result_array = []

    # If the data is ints
    if data_type == INT:

        # Convert each value to an int
        for value in data.split(ARRAY_DELIMITER):
            result_array.append(int(value))

        # Reshape the array
        result_array = array(result_array).reshape(literal_eval(data_shape))

    # If the data is uint8
    elif data_type == UINT8:

        # Convert each value to an uint8
        for value in data.split(ARRAY_DELIMITER):
            result_array.append(uint8(value))

        # Reshape the array
        result_array = array(result_array).reshape(literal_eval(data_shape))

    # If the data is floats
    elif data_type == FLOAT:

        # Convert each value to float
        for value in data.split(ARRAY_DELIMITER):
            result_array.append(float(value))

        # Reshape the array
        result_array = array(result_array).reshape(literal_eval(data_shape))

    # If the data is None
    elif data_type == NONE_PLACEHOLDER:

        # Set it as None
        result_array = None

    # Otherwise raise an exception if the data type is not recognized
    else:
        raise Exception("The data type of '" + data_type + "' was not recognized.")

    # Return the field name and the data
    return split_data[0], result_array


def rebuild_time(split_data: list):

    # Return the field name and a timestamp using the number of seconds and nanoseconds
    return split_data[0], Time(split_data[1], split_data[2])
