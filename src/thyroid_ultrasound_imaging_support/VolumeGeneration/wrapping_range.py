def wrapping_range(index_1: int, index_2: int, source_list: list) -> range:
    """
    Creates a range of numbers for iterating through a list that will properly wrap around the end of the list.

    Parameters
    ----------
    index_1 :
        The index to start the range at.
    index_2 :
        The index to end the range at.
    source_list :
        The list that will be indexed through.

    Returns
    -------
    range :
        A range of indices.
    """

    # If the ending index is less than the starting index,
    if index_2 < index_1:
        # Return a range that uses negative indexing to wrap around
        return range(index_1 - len(source_list), index_2)
    else:
        # Otherwise return a standard range
        return range(index_1, index_2)


