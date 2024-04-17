def calc_index_distance(index_a: int, index_b: int, source_list: list) -> float:
    """
    Calculates the distance between two indices and returns the minimum distance.
    Minimum is calculated with respect to the amount of distance that would
    need to be travelled to reach index A from index B, not by sign.

    A positive distance is returned when index A is greater than index B.

    Parameters
    ----------
    index_a :
        The reference index from which to calculate the distance.
    index_b :
        The index whose distance is not known.
    source_list :
        The list where the indices originate.
    Returns
    -------
    float :
        The distance between index A and B.
    """

    # Calculate the distance between the two indices
    basic_distance = index_a - index_b

    # Calculate the sign of the first distance
    try:
        sign_of_basic_distance = basic_distance / abs(basic_distance)
    except ZeroDivisionError:
        # If the first distance was zero, the sign is positive
        sign_of_basic_distance = 1

    # Add the first distance and the inverse distance to the list
    signed_distances = [basic_distance, int(basic_distance + (-sign_of_basic_distance * len(source_list)))]

    # Calculate the minimum distance by magnitude
    min_distance = min([abs(x) for x in signed_distances])

    # Return the proper distance
    if min_distance in signed_distances:
        return min_distance
    else:
        return -min_distance
