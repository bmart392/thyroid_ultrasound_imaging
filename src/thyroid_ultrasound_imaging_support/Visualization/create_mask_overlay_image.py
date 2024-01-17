"""
Contains the code for the create_mask_overlay_array function.
"""

# Import standard python files
from numpy import newaxis, array, uint8, where
from cv2 import GC_FGD, GC_PR_BGD, GC_BGD

# Import custom python files
from thyroid_ultrasound_imaging_support.ImageFilter.FilterConstants import COLOR_GRAY, COLOR_RGB, COLOR_BGR


# Define the options for overlaying the mask
FADED: int = 0
COLORIZED: int = 1
PREV_IMG_MASK: int = 2


# TODO - MEDIUM - Remove the need for the base_image_num_channels arguement
def create_mask_overlay_array(base_image: array, base_image_num_channels: int,
                              overlay_mask: array, output_image_color: int,
                              overlay_method: int, fade_rate: float = 0.5, overlay_color=(25, 25, 25),
                              color_fg=(0, 35, 0), color_pr_bgd=(35, 0, 0), color_bgd=(0, 0, 0)):
    """
    Returns an array that overlays a binary mask or previous-image mask over an image array.

    Parameters
    ----------
    base_image
        a numpy array containing the base image.
    base_image_num_channels
        the number of color channels in the base image.
    overlay_mask
        a numpy array representing the mask to be overlaid.
    output_image_color
        the desired color sequence of the output image.
    overlay_method
        the method of overlaying the mask on the image.
    fade_rate
        the fade ratio to apply to the inverse of the mask.
        A number less than one darkens the remainder of the image.
    overlay_color
        The color to apply to the masked area formatted in RGB order.
    color_fg
        the color to apply to the foreground of the previous-image mask.
    color_pr_bgd
        the color to apply to the probable background area of the previous-image mask.
    color_bgd
        the color to apply to the background area of the previous-image mask.
    """

    # Options
    # Input image color: gray vs RGB vs BGR
    # Output image color: RGB vs BGR
    # Fade: the part of the image not in the mask is faded an amount
    # Color: the part of the image in the mask is colored

    # Ensure that the image color given has matches the number of channels in the image
    if (base_image_num_channels == 3) and not len(base_image.shape) == 3:
        raise Exception("Base image does not have 3 channels when it is supposed to.")
    elif base_image_num_channels == 1 and not len(base_image.shape) == 2:
        raise Exception("Base image does not have 1 channel when it is supposed to.")
    elif overlay_mask is None:
        raise Exception("Mask cannot be None.")

    # Create the inverse mask
    inverted_mask = 1 - overlay_mask

    # If the image color is 3 channel, update the mask accordingly
    if base_image_num_channels == 3:
        overlay_mask = overlay_mask[:, :, newaxis]
        inverted_mask = inverted_mask[:, :, newaxis]

    # If the output image is being faded,
    if overlay_method == FADED:
        return base_image * overlay_mask + uint8(base_image * inverted_mask * fade_rate)

    # Reverse the color given if BGR color formatting is used
    if output_image_color == COLOR_BGR:
        overlay_color = (overlay_color[2], overlay_color[1], overlay_color[0])
    elif not output_image_color == COLOR_RGB:
        raise Exception("Output color given is not recognized.")

    # If the output image is being colorized,
    if overlay_method == COLORIZED:

        return base_image + uint8(overlay_mask * overlay_color)

    # If the mask is a previous-image mask
    elif overlay_method == PREV_IMG_MASK:
        fg_mask = uint8(where(overlay_mask == GC_FGD, 1, 0))
        pr_bgd_mask = uint8(where(overlay_mask == GC_PR_BGD, 1, 0))
        bgd_mask = uint8(where(overlay_mask == GC_BGD, 1, 0))

        return base_image + uint8(fg_mask * color_fg + pr_bgd_mask * color_pr_bgd + bgd_mask * color_bgd)

    else:
        raise Exception("Image overlay method is not recognized.")
