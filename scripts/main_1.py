# This is a sample Python script.
import cv2 as cv
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.colors import hsv_to_rgb, rgb_to_hsv
from matplotlib import colors
import numpy as np
from ImageData import ImageData
from ImageFilter import ImageFilter

# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    # Read in original image
    original_image = cv.imread('thyroid_us.jpg')

    # Convert to original color to rgb
    rgb_image = cv.cvtColor(original_image, cv.COLOR_BGR2RGB)

    # Set up variable to store all image data
    image_list = []
    current_index = 0

    # Convert rgb image to grayscale, add to list, and increment index
    image_list.append(
        ImageData(
            cv.cvtColor(rgb_image, cv.COLOR_RGB2GRAY),
            "Grayscale Image",
            65,
            95)
    )
    GRAYSCALE_IMAGE_INDEX = current_index
    current_index = current_index + 1

    # Blur grayscale image
    blur_kernel_size = (13, 13)
    image_list.append(
        ImageData(
            cv.GaussianBlur(image_list[GRAYSCALE_IMAGE_INDEX].image_array, blur_kernel_size, 2.5),
            "Grayscale Image" + "\n" + "Gaussian Blurred" + "\n" + blur_kernel_size.__str__() + " Kernel Size",
            55,
            90)
    )
    GRAYSCALE_BLURRED_A_IMAGE_INDEX = current_index
    image_list[GRAYSCALE_BLURRED_A_IMAGE_INDEX].set_blur_kernel(blur_kernel_size)
    current_index = current_index + 1

    # Blur grayscale image
    spatial_window_size = 45
    color_window_size = 45
    image_list.append(
        ImageData(
            cv.cvtColor(cv.pyrMeanShiftFiltering(rgb_image, spatial_window_size, color_window_size), cv.COLOR_RGB2GRAY),
            "Grayscale Image" + "\n" + "Mean Filter Blurred" + "\n" + (spatial_window_size, color_window_size).__str__() + " Parameters",
            55,
            90)
    )
    GRAYSCALE_BLURRED_B_IMAGE_INDEX = current_index
    image_list[GRAYSCALE_BLURRED_B_IMAGE_INDEX].set_blur_kernel(blur_kernel_size)
    current_index = current_index + 1

    """# Blur grayscale image
    blur_kernel_size = (15, 15)
    # grayscale_image_blurred = cv.blur(grayscale_image, blur_kernel_size)
    image_list.append(
        ImageData(
            cv.blur(image_list[GRAYSCALE_IMAGE_INDEX].image_array, blur_kernel_size),
            "Blurred Grayscale Image" + "\n" + blur_kernel_size.__str__() + " Kernel Size",
            55,
            90)
    )
    GRAYSCALE_BLURRED_C_IMAGE_INDEX = current_index
    image_list[GRAYSCALE_BLURRED_C_IMAGE_INDEX].set_blur_kernel(blur_kernel_size)
    current_index = current_index + 1

    # Blur grayscale image
    blur_kernel_size = (20, 20)
    # grayscale_image_blurred = cv.blur(grayscale_image, blur_kernel_size)
    image_list.append(
        ImageData(
            cv.blur(image_list[GRAYSCALE_IMAGE_INDEX].image_array, blur_kernel_size),
            "Blurred Grayscale Image" + "\n" + blur_kernel_size.__str__() + " Kernel Size",
            55,
            90)
    )
    GRAYSCALE_BLURRED_D_IMAGE_INDEX = current_index
    image_list[GRAYSCALE_BLURRED_D_IMAGE_INDEX].set_blur_kernel(blur_kernel_size)
    current_index = current_index + 1
"""
    # Increase contrast in grayscale image
    image_list.append(
        ImageData(
            cv.equalizeHist(image_list[GRAYSCALE_IMAGE_INDEX].image_array),
            "Equalized Grayscale Image",
            150,
            200)
    )
    GRAYSCALE_EQUALIZED_IMAGE_INDEX = current_index
    current_index = current_index + 1

    # Apply Gaussian blur to increased contrast image
    # spatial_window_size = 25
    # color_window_size = 25
    image_list.append(
        ImageData(
            cv.GaussianBlur(image_list[GRAYSCALE_EQUALIZED_IMAGE_INDEX].image_array, blur_kernel_size, 2.5),
            "Equalized Grayscale Image" + "\n" + "Gaussian Blur" + "\n" +
            image_list[GRAYSCALE_BLURRED_A_IMAGE_INDEX].blur_kernel.__str__() + " Kernel Size",
            150,
            200
        )
    )
    GRAYSCALE_EQUALIZED_GAUSSIAN_BLURRED_IMAGE_INDEX = current_index
    image_list[GRAYSCALE_EQUALIZED_GAUSSIAN_BLURRED_IMAGE_INDEX].set_blur_kernel(
        image_list[GRAYSCALE_BLURRED_A_IMAGE_INDEX].blur_kernel)
    current_index = current_index + 1

    # Apply Gaussian blur to increased contrast image
    blur_kernel_size = (13, 13)
    image_list.append(
        ImageData(
            cv.cvtColor(
                cv.pyrMeanShiftFiltering(
                    cv.cvtColor(image_list[GRAYSCALE_EQUALIZED_IMAGE_INDEX].image_array, cv.COLOR_GRAY2RGB),
                    spatial_window_size,
                    color_window_size),
                cv.COLOR_RGB2GRAY),
            "Equalized Grayscale Image" + "\n" + "Mean Filter Blur" + "\n" +
            (spatial_window_size, color_window_size).__str__() + " Parameters",
            125,
            225
        )
    )
    GRAYSCALE_EQUALIZED_MEAN_FILTER_BLURRED_IMAGE_INDEX = current_index
    current_index = current_index + 1

    # Generate two figures for storing images
    image_fig = plt.figure(figsize=(12, 12))
    plot_fig = plt.figure(figsize=(12, 12))

    # Change the active figure
    plt.figure(image_fig.number)

    # Set the number of plots included in the image figure
    num_rows_in_image_fig = 6
    num_cols_in_image_fig = len(image_list)

    # Define an index for tracking the column number
    col_index = 1

    # Plot each image in the image_list
    for image_data in image_list:
        temp_image_axis = image_fig.add_subplot(num_rows_in_image_fig, num_cols_in_image_fig, col_index)
        temp_image_axis.set_title(image_data.image_title, fontsize=6.0)
        if image_list.index(image_data) == 0:
            plt.imshow(cv.pyrMeanShiftFiltering(rgb_image, 13, 13))
        else:
            plt.imshow(image_data.image_array, cmap='gray')
        col_index = col_index + 1

    # Rearrange images in figure to be easier to read
    image_fig.tight_layout()

    # Change the active figure
    plt.figure(plot_fig.number)

    # Set the number of plots included in the image figure
    num_rows_in_plot_fig = 3
    num_cols_in_plot_fig = num_cols_in_image_fig

    # Define an index for tracking the column number
    col_index = 1

    for image_data in image_list:
        temp_plot_axis = plot_fig.add_subplot(num_rows_in_plot_fig, num_cols_in_plot_fig, col_index)
        temp_plot_axis.set_title(image_data.image_title + "\n Histogram", fontsize=6.0)
        temp_plot_axis.hist(image_data.image_array.flatten(), bins=256, range=[0, 256])
        temp_plot_axis.set_xlabel("Intensity", fontsize=6.0)
        temp_plot_axis.set_ylabel("Number of Pixels", fontsize=6.0)
        col_index = col_index + 1

        temp_mask_lower_limit_square = np.full(
            (10, 10, 3), np.full(3, image_data.mask_lower_limit), dtype=np.uint8) / 255.0

        temp_mask_upper_limit_square = np.full(
            (10, 10, 3), np.full(3, image_data.mask_upper_limit), dtype=np.uint8) / 255.0

        temp_mask_limit_axis = plot_fig.add_subplot(num_rows_in_plot_fig, num_cols_in_plot_fig,
                                                    col_index + (num_cols_in_plot_fig - 1))
        temp_mask_limit_axis.set_title("Lower Color Threshold", fontsize=6.0)
        temp_mask_limit_axis.set_xticks([])
        temp_mask_limit_axis.set_yticks([])
        temp_mask_limit_axis.imshow(temp_mask_lower_limit_square)

        temp_mask_limit_axis = plot_fig.add_subplot(num_rows_in_plot_fig, num_cols_in_plot_fig,
                                                    col_index + 2 * num_cols_in_plot_fig - 1)
        temp_mask_limit_axis.set_title("Upper Color Threshold", fontsize=6.0)
        temp_mask_limit_axis.set_xticks([])
        temp_mask_limit_axis.set_yticks([])
        temp_mask_limit_axis.imshow(temp_mask_upper_limit_square)

    # Rearrange plots in figure to be easier to read
    plot_fig.tight_layout()

    # Change the active figure
    plt.figure(image_fig.number)

    # Define an index for tracking the column number
    col_index = 1
    row_index = 1
    # Plot each image in the image_list
    for image_data in image_list:

        # Generate image mask from adaptive thresholding process
        temp_adaptive_mask = cv.adaptiveThreshold(image_data.image_array, 1, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 75, 1)
        MASK_PROFILE_A = 0

        # Generate image mask from image specific values
        temp_basic_mask = cv.inRange(image_data.image_array, image_data.mask_lower_limit, image_data.mask_upper_limit)
        MASK_PROFILE_B = 1

        # Store masks
        all_masks = (("Adaptive Threshold", temp_adaptive_mask, MASK_PROFILE_A),
                     ("Basic Threshold", temp_basic_mask, MASK_PROFILE_B))

        for mask_tuple in all_masks:

            individual_mask_text = mask_tuple[0]
            individual_mask = mask_tuple[1]
            individual_mask_profile = mask_tuple[2]

            # Plot original mask generated from thresholding
            temp_image_axis = image_fig.add_subplot(num_rows_in_image_fig, num_cols_in_image_fig,
                                                          col_index + row_index * num_cols_in_image_fig)
            temp_image_axis.set_title(individual_mask_text + "\nUnmodified Mask", fontsize=6.0)
            temp_image_axis.set_xticks([])
            temp_image_axis.set_yticks([])
            plt.imshow(individual_mask, cmap='gray')
            row_index = row_index + 1

            # Modify mask
            if individual_mask_profile == MASK_PROFILE_A:
                """individual_mask = cv.morphologyEx(
                    individual_mask,
                    cv.MORPH_OPEN,
                    np.ones((3, 3), np.uint8),
                    iterations=3)
                individual_mask = cv.morphologyEx(
                    individual_mask,
                    cv.MORPH_CLOSE,
                    np.ones((3, 3), np.uint8),
                    iterations=2)
                individual_mask = cv.morphologyEx(
                    individual_mask,
                    cv.MORPH_OPEN,
                    np.ones((4, 4), np.uint8),
                    iterations=3) # best result (4,4) and 9
                individual_mask = cv.morphologyEx(
                    individual_mask,
                    cv.MORPH_CLOSE,
                    np.ones((4, 4), np.uint8),
                    iterations=1)"""
                individual_mask = cv.morphologyEx(
                    individual_mask,
                    cv.MORPH_RECT,
                    np.ones((4, 4), np.uint8),
                    iterations=2)  # best result (4,4) and 9

            elif individual_mask_profile == MASK_PROFILE_B:
                """individual_mask = cv.morphologyEx(
                    individual_mask,
                    cv.MORPH_CLOSE,
                    np.ones((1, 1), np.uint8),
                    iterations=1)
                individual_mask = cv.morphologyEx(
                    individual_mask,
                    cv.MORPH_OPEN,
                    np.ones((4, 4), np.uint8),
                    iterations=11)  # best result (4,4) and 9
                individual_mask = cv.morphologyEx(
                    individual_mask,
                    cv.MORPH_CLOSE,
                    np.ones((1, 1), np.uint8),
                    iterations=1)"""
                individual_mask = cv.morphologyEx(
                    individual_mask,
                    cv.MORPH_RECT,
                    np.ones((4, 4), np.uint8),
                    iterations=2)
                individual_mask = cv.morphologyEx(
                    individual_mask,
                    cv.MORPH_CLOSE,
                    np.ones((4, 4), np.uint8),
                    iterations=3)
                individual_mask = cv.morphologyEx(
                    individual_mask,
                    cv.MORPH_ERODE,
                    np.ones((4, 4), np.uint8),
                    iterations=6)

            individual_mask_contours, heirarchy = cv.findContours(individual_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
            individual_mask_contours = sorted(individual_mask_contours, key=cv.contourArea, reverse=True)
            contour_centroids = []
            if len(individual_mask_contours) != 0:
                largest_contour = [individual_mask_contours[0]]

                temp_moments = cv.moments(largest_contour[0])
                temp_centroid_x = int(temp_moments["m10"] / temp_moments["m00"])
                temp_centroid_y = int(temp_moments["m01"] / temp_moments["m00"])

                contour_centroids.append((temp_centroid_x, temp_centroid_y))

                area_of_largest_contour = cv.contourArea(individual_mask_contours[0])

                if len(individual_mask_contours) > 1:
                    area_of_second_largest_contour = cv.contourArea(individual_mask_contours[1])

                    if (area_of_largest_contour - area_of_second_largest_contour) < .1 * area_of_largest_contour:
                        largest_contour.append(individual_mask_contours[1])

                        temp_moments = cv.moments(largest_contour[1])
                        temp_centroid_x = int(temp_moments["m10"] / temp_moments["m00"])
                        temp_centroid_y = int(temp_moments["m01"] / temp_moments["m00"])

                        contour_centroids.append((temp_centroid_x, temp_centroid_y))

                    individual_mask = np.zeros(individual_mask.shape, np.uint8)
                    individual_mask = cv.drawContours(individual_mask, largest_contour, -1, 255, -1)
                    """for each_contour in largest_contour:
                        for each_pixel in each_contour:
                            each_pixel = each_pixel[0]
                            individual_mask[each_pixel[1]][each_pixel[0]] = 255"""
            else:
                largest_contour = []

            """ ret, labelled_regions_individual_mask = cv.connectedComponents(individual_mask)
            labelled_regions_individual_mask = labelled_regions_individual_mask + 1

            individual_mask = cv.morphologyEx(individual_mask, cv.MORPH_DILATE, np.ones((10,10), np.uint8), iterations=8)"""


            # Plot modified mask
            temp_image_axis = image_fig.add_subplot(num_rows_in_image_fig, num_cols_in_image_fig,
                                                    col_index + row_index * num_cols_in_image_fig)
            temp_image_axis.set_title(individual_mask_text + "\nModified Mask", fontsize=6.0)
            temp_image_axis.set_xticks([])
            temp_image_axis.set_yticks([])
            plt.imshow(individual_mask, cmap='gray')
            row_index = row_index + 1

        # Generate masked image from mask and original image
        temp_masked_result = cv.bitwise_and(cv.cvtColor(image_data.image_array, cv.COLOR_GRAY2RGB),
                                            cv.cvtColor(image_data.image_array, cv.COLOR_GRAY2RGB),
                                            mask=individual_mask)
        temp_rgb_image = cv.cvtColor(cv.imread('thyroid_us.jpg'), cv.COLOR_BGR2RGB)
        temp_masked_result = cv.drawContours(temp_rgb_image, largest_contour, -1, (0, 255, 0), -1)


        for each_centroid in contour_centroids:
            temp_masked_result = cv.circle(temp_masked_result, each_centroid, 6, (255,255,255), -1)

        temp_image_axis = image_fig.add_subplot(num_rows_in_image_fig, num_cols_in_image_fig,
                                                col_index + row_index * num_cols_in_image_fig)
        temp_image_axis.set_title("Masked Image", fontsize=6.0)
        temp_image_axis.set_xticks([])
        temp_image_axis.set_yticks([])
        plt.imshow(temp_masked_result)
        row_index = row_index + 1

        col_index = col_index + 1
        row_index = 1

    # Rearrange images in figure to be easier to read
    image_fig.tight_layout()

    plt.savefig("segmented_images" + ".png")
    plt.figure(plot_fig.number)
    plt.savefig("color_plots" + ".png")




