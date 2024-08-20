"""
Contains the code for the create_dice_score_mask function.
"""

# Import standard python packages
from copy import copy
from numpy import ndarray, ones, load, uint8
from cv2 import morphologyEx, MORPH_ERODE, MORPH_DILATE, imshow, waitKey

# Import custom python packages
from thyroid_ultrasound_imaging_support.Validation.calculate_dice_score import calculate_dice_score


def create_dice_score_mask(original_mask: ndarray,
                           dice_score_to_achieve: float,
                           given_kernel_size: tuple = None,
                           mandate_minimum_one_iteration: bool = False) -> ndarray:
    """
    Creates an image mask with the desired DICE score relative to the original mask by eroding the original mask.

    Parameters
    ----------
    original_mask :
        The image mask to erode.
    dice_score_to_achieve :
        The DICE score that should be achieved.
    given_kernel_size :
        The size of the kernel to use to morph the given mask
    mandate_minimum_one_iteration :
        Requires that the mask is shrunk or grown by at least one iteration
        even if it results in a less accurate DICE score.

    Returns
    -------
    ndarray
        An image mask with the desired DICE score relative to the original image mask.
    """

    # Verify that the image mask given has the correct attributes
    if not len(original_mask.shape) == 2:
        raise Exception("Image mask has " + str(len(original_mask.shape)) +
                        " dimensions but can only have 2 dimensions.")
    if not original_mask.dtype == uint8:
        raise Exception("Image mask must have data type of uin8. Mask given has data type of " +
                        str(original_mask.dtype))
    if original_mask.max() > 1 or original_mask.min() < 0:
        raise Exception("Image mask contains values other than 0 and 1.")

    # Verify that the DICE score to achieve is valid
    if dice_score_to_achieve <= 0.0:
        raise Exception("DICE score given of " + str(dice_score_to_achieve) + " is less than 0.")

    if given_kernel_size is not None:
        kernel_size_to_use = given_kernel_size
        if given_kernel_size[0] % 2 == 1 and given_kernel_size[1] % 2 == 1:
            anchor_point = (int((given_kernel_size[0] - 1) / 2), int((given_kernel_size[1] - 1) / 2))
        else:
            anchor_point = None
    else:
        kernel_size_to_use = (3, 3)
        anchor_point = (1, 1)

    # Define the inverse dice score to use when the desired mask should be an expansion of the original mask
    inverse_dice_score_to_achieve = 1 / dice_score_to_achieve

    # Capture if the resulting mask should be smaller than the original mask
    if dice_score_to_achieve <= 1.:
        shrink_mask = True
    else:
        shrink_mask = False

    # Create a copy of the original mask to modify
    current_mask = copy(original_mask)

    # Define the current DICE score between the two masks
    current_mask_score = 1.0

    # Define flag to know when to stop the iteration loop
    if shrink_mask:
        continue_iterating_flag = dice_score_to_achieve <= current_mask_score
    else:
        continue_iterating_flag = inverse_dice_score_to_achieve <= current_mask_score

    # Define an iteration counter
    iteration_counter = 0

    # While the mask has not reached the desired DICE score,
    while continue_iterating_flag:

        # Save the current mask and its score
        previous_result_mask = current_mask
        previous_result_mask_score = current_mask_score

        if shrink_mask:
            # Shrink the current mask
            current_mask = morphologyEx(current_mask,
                                        MORPH_ERODE,
                                        ones(kernel_size_to_use, current_mask.dtype),
                                        iterations=1,
                                        anchor=anchor_point)

            # Recalculate the score of the updated mask
            current_mask_score = calculate_dice_score(original_mask, current_mask)
        else:
            # Expand the current mask
            current_mask = morphologyEx(current_mask,
                                        MORPH_DILATE,
                                        ones(kernel_size_to_use, current_mask.dtype),
                                        iterations=1,
                                        anchor=anchor_point)

            # Recalculate the score of the updated mask
            current_mask_score = calculate_dice_score(current_mask, original_mask)

        # Update the flag
        if shrink_mask:
            continue_iterating_flag = dice_score_to_achieve <= current_mask_score
        else:
            continue_iterating_flag = inverse_dice_score_to_achieve <= current_mask_score

        # Increment the iteration counter
        iteration_counter = iteration_counter + 1

    # Choose the mask with the score closest to the desired score to return
    if shrink_mask:
        comparison_score = dice_score_to_achieve
    else:
        comparison_score = inverse_dice_score_to_achieve

    if mandate_minimum_one_iteration and iteration_counter == 1:
        result_mask = current_mask
    elif abs(previous_result_mask_score - comparison_score) < abs(current_mask_score - comparison_score):
        result_mask = previous_result_mask
    else:
        result_mask = current_mask

    return result_mask


if __name__ == '__main__':

    # Load the mask to test
    mask = load('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/'
                'Test/Experimentation/ground_truth.npy')

    # Show the result
    imshow('result', mask * uint8(125) + create_dice_score_mask(mask, 1.5) * uint8(50))
    waitKey(-1)
