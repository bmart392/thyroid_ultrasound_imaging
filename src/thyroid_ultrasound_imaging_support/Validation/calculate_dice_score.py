"""
Contains the code for the function calculate_dice_score
"""

# Import standard python packages
from numpy import ndarray, sum, where


def calculate_dice_score(array_one: ndarray, array_two: ndarray) -> float:
    """
    Calculates the DICE score between two numpy arrays of the same size.
    Parameters
    ----------
    array_one :
        The first array to use to calculate the DICE score.

    array_two :
        The second array to use to calculate the DICE score.

    Returns
    -------
    float : The DICE score of the two arrays
    """
    # Ensure that the arrays match in shape before continuing
    if not array_one.shape == array_two.shape:
        raise Exception("Array shapes do not match.")

    # Calculate the union of the two arrays
    union = array_one + array_two

    # Ensure that union only contains the expected values
    if sum(where(union > 2, 1, 0)) > 0:
        raise Exception("Arrays contain values other than 1 and 0.")

    # Return the DICE score
    return 2 * sum(where(union == 2, 1, 0)) / sum(union)
