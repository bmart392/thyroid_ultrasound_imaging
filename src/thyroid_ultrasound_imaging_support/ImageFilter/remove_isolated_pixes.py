"""
File containing the remove_isolated_pixes function definition.
"""

# Import standard python packages
from numpy import array, ndarray


# Import custom python packages


def remove_isolated_pixes(mask: ndarray) -> None:
    """
    Removes isolated pixels from the given mask. An isolated cell is considered a cell that has than two neighbor
    cells in the mask.

    Parameters
    ----------
    mask :
        The mask in which to remove the isolated pixels
    """

    # Ensure that the given mask has only 2 dimensions
    if len(mask.shape) != 2:
        raise Exception("Incorrect mask size given.")

    # Iterate through each cell in the mask,
    for row in range(mask.shape[0]):
        for column in range(mask.shape[1]):

            # If the given cell is part of the mask,
            if mask[row, column] == 1:

                # Calculate the sum of the cells adjacent to the current cell
                try:
                    this_slice_sum = sum(sum(mask[row - 1:row + 2, column - 1:column + 2]))

                except TypeError:
                    this_slice_sum = sum(sum(mask[max(row - 1, 0):min(row + 2, mask.shape[0]),
                                 max(column - 1, 0):min(column + 2, mask.shape[1])]))

                # If 2 or more neighbor cells are not included in the mask, remove the current cell from the mask
                if this_slice_sum < 3:
                    mask[row, column] = 0


if __name__ == '__main__':

    # Create two arrays
    original_array = array([[1, 0, 0, 0, 0],
                            [0, 1, 1, 1, 1],
                            [0, 0, 0, 1, 1],
                            [0, 1, 1, 0, 1],
                            [0, 0, 1, 1, 1]])
    test_array = array([[1, 0, 0, 0, 0],
                        [0, 1, 1, 1, 1],
                        [0, 0, 0, 1, 1],
                        [0, 1, 1, 0, 1],
                        [0, 0, 1, 1, 1]])

    # Remove the isolated pixels
    remove_isolated_pixes(test_array)

    # Compare the result with the original
    print(original_array + test_array)
