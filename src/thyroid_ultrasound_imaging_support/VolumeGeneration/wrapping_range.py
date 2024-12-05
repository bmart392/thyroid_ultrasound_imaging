from numpy import linspace

def wrapping_range(index_1: int, index_2: int, source_list: list) -> list:
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

    # Create the default best option
    best_option = [int(x) for x in linspace(index_1, index_2, abs(index_1 - index_2), endpoint=False)]

    # Create the second-best option which accounts for wrapping around the end of the list
    second_best_option = list(range(index_1 - len(source_list), index_2))

    # Create the third best option
    third_best_option = [int(x) for x in linspace(index_2 - (len(source_list) - 1), index_1,
                                                  len(source_list) - abs(index_1 - index_2), endpoint=False)]

    # Return the list that includes fewer points
    options = [best_option, second_best_option, third_best_option]
    option_lengths = [len(option) for option in options]
    return options[option_lengths.index(min(option_lengths))]


