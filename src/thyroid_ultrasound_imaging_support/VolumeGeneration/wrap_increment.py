"""
File containing the wrap_increment function definition.
"""


# Import standard python packages

# Import custom python packages


def wrap_increment(index: int, source_list_len: int, increment_amount: int = 1) -> int:
    """
    Increments the given index by the given value but accounts for wrapping around the end of the list.

    Parameters
    ----------
    index :
        The index that should be incremented.
    source_list_len :
        The length of the list in which the index will be used.
    increment_amount :
        The amount by which the index will be incremented.
    Returns
    -------
    int :
        The updated index.
    """

    # Increment the index
    index = index + increment_amount

    # If the index is larger than the list,
    if index >= source_list_len:

        # Subtract the length of the list
        index = index - source_list_len

    # Return the index
    return index


if __name__ == '__main__':
    pass
