"""
Contains the code for the create_mask_overlay_array function.
"""

# Import standard python files
from numpy import newaxis, array, uint8, where, ones
from cv2 import GC_FGD, GC_BGD, GC_PR_FGD, cvtColor, \
    COLOR_GRAY2RGB, COLOR_RGB2BGR, COLOR_BGR2RGB, imread, imshow, waitKey

# Import custom python files
from thyroid_ultrasound_imaging_support.ImageFilter.FilterConstants import COLOR_GRAY, COLOR_RGB, COLOR_BGR

# Define the options for overlaying the mask
FADED: int = 0
COLORIZED: int = 1
PREV_IMG_MASK: int = 2


def create_mask_overlay_array(base_image: array, overlay_mask: array,
                              base_image_color: int = COLOR_RGB, output_image_color: int = COLOR_RGB,
                              overlay_method: int = FADED, fade_rate: float = 0.5, overlay_color=(25, 25, 25),
                              color_fgd=(0, 35, 0), color_pr_fgd=(35, 0, 0), color_bgd=(0, 0, 0)):
    """
    Returns an array that overlays a binary mask or previous-image mask over an image array.

    Parameters
    ----------
    base_image
        a numpy array containing the base image.
    base_image_color
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
    color_fgd
        the color to apply to the foreground of the previous-image mask.
    color_pr_fgd
        the color to apply to the probable foreground area of the previous-image mask.
    color_bgd
        the color to apply to the background area of the previous-image mask.
    """

    # Options
    # Input image color: gray vs RGB vs BGR
    # Output image color: RGB vs BGR
    # Fade: the part of the image not in the mask is faded an amount
    # Color: the part of the image in the mask is colored

    # Ensure that the image color given has matches the number of channels in the image
    if not base_image.shape[0:2] == overlay_mask.shape[0:2]:
        raise Exception("Base image has shape " + str(base_image.shape[0:2]) + " but mask has shape " +
                        str(overlay_mask.shape[0:2]) + ".")
    elif not len(overlay_mask.shape) == 2:
        raise Exception("Mask has incorrect dimensions.")
    elif overlay_mask is None:
        raise Exception("Mask cannot be None.")

    # Create the inverse mask
    inverted_mask = 1 - overlay_mask

    # If the image is 1D
    if len(base_image.shape) == 2:

        # Overlay the mask using the fade rate
        result_image = base_image * overlay_mask + uint8(base_image * inverted_mask * fade_rate)

        # Convert the image to a 3 channel image if required
        if output_image_color == COLOR_RGB or output_image_color == COLOR_BGR:
            result_image = cvtColor(result_image, COLOR_GRAY2RGB)

    # If the image is a 3 channel image
    elif len(base_image.shape) == 3 and base_image.shape[2] == 3:

        # Add an extra dimension to the mask
        overlay_mask = overlay_mask[:, :, newaxis]
        inverted_mask = inverted_mask[:, :, newaxis]

        # If the output image is being faded,
        if overlay_method == FADED:
            result_image = base_image * overlay_mask + uint8(base_image * inverted_mask * fade_rate)

        # If the output image is being colorized,
        elif overlay_method == COLORIZED:

            # If the base image is in RGB order,
            if base_image_color == COLOR_RGB:

                # Create the result mask using the overlay color in RGB
                result_image = base_image + uint8(overlay_mask * overlay_color)

                # If the output needs to be in BGR order,
                if output_image_color == COLOR_BGR:
                    # Convert the result to BGR order
                    result_image = cvtColor(result_image, COLOR_RGB2BGR)

            # if the output image is in BGR order,
            elif base_image_color == COLOR_BGR:

                # Convert the overlay color to BGR order
                overlay_color = (overlay_color[2], overlay_color[1], overlay_color[0])

                # Create the result mask in BGR
                result_image = base_image + uint8(overlay_mask * overlay_color)

                # If the output needs to be in RGB,
                if output_image_color == COLOR_RGB:
                    # Convert the result to BGR order
                    result_image = cvtColor(result_image, COLOR_BGR2RGB)

            else:
                raise Exception("Base image color of " + str(base_image_color) + " was not recognized.")

        # If a previous-image mask is being visualized,
        elif overlay_method == PREV_IMG_MASK:

            # Split up the mask into its various components
            fg_mask = uint8(where(overlay_mask == GC_FGD, 1, 0))
            pr_fgd_mask = uint8(where(overlay_mask == GC_PR_FGD, 1, 0))
            bgd_mask = uint8(where(overlay_mask == GC_BGD, 1, 0))

            # If the base image is in RGB order,
            if base_image_color == COLOR_RGB:

                # Create the result mask using the fg, bgd, and pr_fgd colors in RGB
                result_image = base_image + \
                               uint8(fg_mask * color_fgd + pr_fgd_mask * color_pr_fgd + bgd_mask * color_bgd)

                # If the output needs to be in BGR order,
                if output_image_color == COLOR_BGR:
                    # Convert the result to BGR order
                    result_image = cvtColor(result_image, COLOR_RGB2BGR)

            # If the base image is in BGR order,
            elif base_image_color == COLOR_BGR:
                # Put the colors in BGR order
                color_fgd = (color_fgd[2], color_fgd[1], color_fgd[1])
                color_bgd = (color_bgd[2], color_bgd[1], color_bgd[1])
                color_pr_fgd = (color_pr_fgd[2], color_pr_fgd[1], color_pr_fgd[1])

                # Create the result mask using the fg, bgd, and pr_fgd colors in RGB
                result_image = base_image + \
                               uint8(fg_mask * color_fgd + pr_fgd_mask * color_pr_fgd + bgd_mask * color_bgd)

                # If the output needs to be in RGB order,
                if output_image_color == COLOR_BGR:
                    # Convert the result to RGB order
                    result_image = cvtColor(result_image, COLOR_BGR2RGB)

            else:
                raise Exception("Base image color of " + str(base_image_color) + " was not recognized.")

        else:
            raise Exception("Image overlay method is not recognized.")
    else:
        raise Exception("Base image has " + str(len(base_image.shape)) +
                        " dimensions. Only 2D or 3D arrays are allowed. Base image has " + str(base_image.shape[2]) +
                        " channels.")

    # Return the result image
    return result_image


if __name__ == '__main__':
    # Load the image
    test_image = imread('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/Test/Experimentation/'
                             'Experiment_2024-01-12/Images/Slice_00090.png').astype(uint8)

    # image = cvtColor(image, COLOR_RGB2GRAY)

    # Create a mask to overlay
    test_mask = ones(test_image.shape[0:2], dtype=uint8)

    # Create composite image
    test_result_image = create_mask_overlay_array(test_image, test_mask, base_image_color=COLOR_GRAY,
                                                  output_image_color=COLOR_GRAY,
                                                  overlay_method=COLORIZED, overlay_color=(25, 0, 0))

    # Display the image (matplotlib uses RGB encoding)
    imshow("test",test_result_image)
    waitKey(-1)
    # show()
