"""
Contains the code for the validate_transformation_matrix function.
"""
# TODO - Dream - Make this more robust and complete

# Import standard python packages
from numpy import ndarray


def validate_transformation_matrix(transformation_matrix: ndarray):
    """
    Checks that the given transformation is a valid homogeneous transformation matrix.

    Parameters
    ----------
    transformation_matrix :
        The transformation matrix to check.

    Returns
    -------
    bool
        Returns if the transformation matrix is valid.
    """
    return transformation_matrix.shape == (4, 4)
