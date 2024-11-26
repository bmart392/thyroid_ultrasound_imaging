"""
Contains code for creating single line representations of data.
"""

# Import standard python packages
from numpy import array, product, uint8, ndarray, int32, int64
from rospy import Time
from genpy.rostime import Time as genpyTime
from ast import literal_eval

# Define a placeholder value for none values
NONE_PLACEHOLDER: str = "NONE"

# Define the delimiter placed between values in an array
ARRAY_DELIMITER: str = ","

# Define the delimiter placed between entries in a list
LIST_DELIMITER: str = "|"

# Define the delimiter placed between values in a tuple
TUPLE_DELIMITER: str = "!"

# Define the delimiter character
DELIMITER: str = ":"

# Define the end of line character
END_OF_LINE: str = ""

# Define the new line character
NEW_LINE: str = "\n"

# Define values for the different data types
INT: str = "int"
INT32: str = "int32"
INT64: str = "int64"
UINT8: str = "uint8"
FLOAT: str = "float"
STRING: str = "str"

# Define values for the different types of single lines
SIMPLE: str = "simple"
ARRAY: str = "array"
LIST: str = "list"
TUPLE: str = "tuple"
TIME: str = "time"


def create_single_line_simple_data(field_name: str, data, previous_string: str) -> str:
    """
    Creates a single line string representation of a string, integer, float, or None object.
    The single line is added on to the previous string and separated by a carriage return.
    The single line string is always formatted as "<field_name>:<data>\n".
    A placeholder value is inserted into the string whenever a None type is given.
    """

    # Convert data to a string if it is an int
    if type(data) == int:
        data = str(data)
        data_type = INT

    # Convert the data to a string if it is an int
    elif type(data) == float:
        data = str(data)
        data_type = FLOAT

    # Convert the data to a string if it is None
    elif data is None:
        data = NONE_PLACEHOLDER
        data_type = NONE_PLACEHOLDER

    else:

        # If the data is not a string, raise an exception
        if not type(data) == str:
            raise Exception("The class of the data field is not a string, it is a " + str(type(data)))

        # Otherwise set the data type as string
        data_type = STRING

    # Otherwise return the string form
    return (previous_string + SIMPLE + DELIMITER + field_name + DELIMITER + data_type +
            DELIMITER + data + END_OF_LINE + NEW_LINE)


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
        if not type(data) == ndarray:
            # Raise an exception to the user if it is not an array
            raise Exception("The class of the data field is not an array, it is a " + str(type(data)))

        # Determine the data type of the data within the array
        if data.dtype == int:
            data_type = INT
        elif data.dtype == int32:
            data_type = INT32
        elif data.dtype == int64:
            data_type = INT64
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
        string_data = string_data[:-len(ARRAY_DELIMITER)]

        # Determine the shape of the original data
        data_shape = str(data.shape)

    # Return the string form of the data
    return (previous_string + ARRAY + DELIMITER + field_name + DELIMITER + data_type + DELIMITER + data_shape +
            DELIMITER + string_data + END_OF_LINE + NEW_LINE)


def create_single_line_list_data(field_name: str, data: list, previous_string: str) -> str:
    """
    Creates a single line representation of a list.
    The single line is added on to the previous string and separated by a carriage return.
    The single line string is always formatted as "<field_name>:<data.shape>:<data>\n".
    A placeholder value is inserted into the string whenever a None type is given.
    """

    # If the data is None
    if data is None:

        # Replace it with the placeholder value
        string_data = NONE_PLACEHOLDER

    else:

        # Add the string delimiter to show that the list items are beginning
        string_data = ""

        # Ensure the data is a list before continuing
        if not type(data) == list:
            # Raise an exception to the user if it is not a list
            raise Exception("The class of the data field is not a list, it is a " + str(type(data)))

        # Set an iterator to use to name each element in the array
        ii = 0

        # For each item in the list
        for list_item in data:

            # Check if the current list item is an array
            if type(list_item) == ndarray:

                # If it is, use the array generation method but remove the end of line and new line characters
                string_data = create_single_line_array_data(str(ii), list_item, string_data)[:-len(END_OF_LINE +
                                                                                                   NEW_LINE)]

            # Check if the current list item is a list
            elif type(list_item) == list:

                # If it is, use the list generation method but remove the end of the line and the new line characters
                string_data = create_single_line_list_data(str(ii), list_item, string_data)[:-len(END_OF_LINE +
                                                                                                  NEW_LINE)]

            # if the current list item is a tuple
            elif type(list_item) == tuple:

                # Determine the data type of the data within the tuple
                if type(list_item[0]) == int:
                    data_type = INT
                elif type(list_item[0]) == int32:
                    data_type = INT32
                elif type(list_item[0]) == int64:
                    data_type = INT64
                elif type(list_item[0]) == uint8:
                    data_type = UINT8
                elif type(list_item[0]) == float:
                    data_type = FLOAT

                # Raise an exception if the data type is not recognized
                else:
                    raise Exception("The type of the data field " + str(type(list_item[0])) + " was not recognized.")

                # Add the tuple tag and data type to the string
                string_data = string_data + TUPLE + DELIMITER + str(ii) + DELIMITER + data_type + DELIMITER

                # Add each value from the tuple to the string data
                for tuple_item in list_item:
                    string_data = string_data + str(tuple_item) + TUPLE_DELIMITER

                # Remove the extra delimiter
                string_data = string_data[:-len(TUPLE_DELIMITER)]

            # Otherwise raise an exception because an unexpected data type was in the list
            else:
                raise Exception("The class of the data contained within the list is not an array or a tuple, "
                                "it is a " + str(type(data[0])))

            # Increment the iterator
            ii = ii + 1

            # Define a variable for the string form of the data
            string_data = string_data + LIST_DELIMITER

    # Remove the extra list delimiter from the end
    string_data = string_data[:-len(LIST_DELIMITER)]

    # Return the string form of the data
    return previous_string + LIST + DELIMITER + field_name + LIST_DELIMITER + string_data + END_OF_LINE + NEW_LINE


def create_single_line_time_stamp(field_name: str, data: Time, previous_string: str) -> str:
    """
    Creates a single line string representation of a time stamp.
    The single line is added on to the previous string and separated by a carriage return.
    The single line string is always formatted as "<field_name>:<data.secs>:<data.nsecs>\n".
    """

    # If the data field is not a time object
    if isinstance(data, Time) or isinstance(data, genpyTime):
        # Return the string form of the data
        return (previous_string + TIME + DELIMITER + field_name + DELIMITER + str(data.secs) +
                DELIMITER + str(data.nsecs) + END_OF_LINE + NEW_LINE)

    # Raise an exception to the user
    raise Exception("The class of the data field is not " + str(type(Time.now())) + ", it is " + str(type(data)))


def rebuild_data(single_line: str):
    """
    Rebuilds data from a single line of text. The single line must start with the type of data contained within.
    """

    # Pull out the type of data stored in the single line
    single_line_type = single_line[:single_line.index(DELIMITER)]

    # Rebuild list data using the appropriate function
    if single_line_type == LIST:
        return rebuild_list_data(single_line[len(LIST) + len(DELIMITER):])

    else:
        # Split up the data based on the known data delimiter
        split_data = single_line.split(DELIMITER)

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
    data = data.replace(NEW_LINE, "")

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
    data = data.replace(NEW_LINE, "")

    # Define a temporary list to store the result
    result_array = []

    # If the data is ints
    if data_type == INT:

        # Convert each value to an int
        for value in data.split(ARRAY_DELIMITER):
            result_array.append(int(value))

        # Reshape the array
        result_array = array(result_array).reshape(literal_eval(data_shape))

    # If the data is int32
    elif data_type == INT32:

        # Convert each value to an int32
        for value in data.split(ARRAY_DELIMITER):
            result_array.append(int32(value))

        # Reshape the array
        result_array = array(result_array).reshape(literal_eval(data_shape))

    # If the data is int64
    elif data_type == INT64:

        # Convert each value to an int64
        for value in data.split(ARRAY_DELIMITER):
            result_array.append(int64(value))

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
    try:
        return split_data[0], Time(int(split_data[1]), int(split_data[2]))
    except ValueError:
        return split_data[0], Time(int(float(split_data[1])), int(float(split_data[2])))

def rebuild_list_data(string_data: str):

    # Split the data using the list delimiter
    split_data = string_data.split(LIST_DELIMITER)

    # Pop out the field name
    field_name = split_data.pop(0)

    # Define a list to return
    return_list = []

    # Rebuild the list based on the type of data stored within
    for list_item_string in split_data:

        # Split up the list item string based on the known data delimiter
        list_item_split = list_item_string.split(DELIMITER)

        if list_item_split[0] == ARRAY:
            _, list_item_object = rebuild_array_data(list_item_split[1:])
        elif list_item_split[0] == TUPLE:
            _, list_item_object = rebuild_tuple_data(list_item_split[1:])
        elif list_item_split[0] == LIST:
            # TODO - HIGH - This seems broken but I think I simply have never actually had this code try to run
            _, list_item_object = rebuild_list_data(list)
        elif list_item_split[0] == "":
            return_list = []
            break
        else:
            raise Exception(list_item_split[0] + " is not a recognized data type.")

        return_list.append(list_item_object)

    return field_name, return_list


def rebuild_tuple_data(split_data: list):
    # Pull out the data type, data shape, the data from the string
    data_type = split_data[1]
    data = split_data[2]

    # Remove the carriage return if it is included in the data
    data = data.replace(NEW_LINE, "")

    # Define a temporary list to store the result
    result_tuple = []

    # If the data is ints
    if data_type == INT:

        # Convert each value to an int
        for value in data.split(TUPLE_DELIMITER):
            result_tuple.append(int(value))

        # Convert the list to a tuple
        result_tuple = tuple(result_tuple)

    # If the data is int32
    elif data_type == INT32:

        # Convert each value to an int32
        for value in data.split(TUPLE_DELIMITER):
            result_tuple.append(int32(value))

        # Convert the list to a tuple
        result_tuple = tuple(result_tuple)

    # If the data is int64
    elif data_type == INT64:

        # Convert each value to an int64
        for value in data.split(TUPLE_DELIMITER):
            result_tuple.append(int64(value))

        # Convert the list to a tuple
        result_tuple = tuple(result_tuple)

    # If the data is uint8
    elif data_type == UINT8:

        # Convert each value to an uint8
        for value in data.split(ARRAY_DELIMITER):
            result_tuple.append(uint8(value))

        # Convert the list to a tuple
        result_tuple = tuple(result_tuple)

    # If the data is floats
    elif data_type == FLOAT:

        # Convert each value to float
        for value in data.split(ARRAY_DELIMITER):
            result_tuple.append(float(value))

        # Convert the list to a tuple
        result_tuple = tuple(result_tuple)

    # If the data is None
    elif data_type == NONE_PLACEHOLDER:

        # Set it as None
        result_tuple = None

    # Otherwise raise an exception if the data type is not recognized
    else:
        raise Exception("The data type of '" + data_type + "' was not recognized.")

    # Return the field name and the data
    return split_data[0], result_tuple
