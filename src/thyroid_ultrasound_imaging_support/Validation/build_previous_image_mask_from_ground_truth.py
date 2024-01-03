"""
Contains the code for the build_previous_image_mask_from_ground_truth function.
"""
# Import standard python packages
from cv2 import GC_PR_BGD, GC_FGD, imshow, waitKey
from numpy import load, uint8

# Import custom python packages
from thyroid_ultrasound_imaging_support.Validation.create_dice_score_mask import create_dice_score_mask


def build_previous_image_mask_from_ground_truth(ground_truth_mask,
                                                desired_foreground_accuracy: float = 1.0,
                                                desired_probable_background_expansion_factor: float = 0.5):
    """
    Builds a previous_image_mask from a ground truth segmentation where the foreground has a desired DICE score and
    the probable background is expanded by the factor given.

    Parameters
    ----------
    ground_truth_mask :
        An array containing the correct segmentation of the image.
    desired_foreground_accuracy :
        The desired DICE score for the foreground of the mask.
    desired_probable_background_expansion_factor :
        The percentage increase of the foreground used to generate the probable background.

    """
    # Create the mask for the foreground
    foreground_mask = create_dice_score_mask(ground_truth_mask, desired_foreground_accuracy)

    # Build the probable background from the foreground mask
    foreground_plus_probable_background_mask = create_dice_score_mask(foreground_mask, 1 +
                                                                      desired_probable_background_expansion_factor)

    # Return the combination of the two arrays
    return (foreground_plus_probable_background_mask - foreground_mask) * GC_PR_BGD + foreground_mask * GC_FGD


if __name__ == '__main__':

    # Load the mask to test
    mask = load('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/'
                'Test/Experimentation/ground_truth.npy')

    # Show the result
    imshow('result', build_previous_image_mask_from_ground_truth(mask, 0.8, 0.5)*uint8(125))
    waitKey(-1)