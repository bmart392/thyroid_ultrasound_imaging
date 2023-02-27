import cv2
import matplotlib.pyplot as plt
from numpy import array, zeros, sum, uint8
import os


class ImageData:
    """def __init__(self, src_image, image_title, lower_limit, upper_limit):
        self.image_array = src_image
        self.image_title = image_title
        self.blur_kernel = None
        self.mask_lower_limit = lower_limit
        self.mask_upper_limit = upper_limit
        self.original_image=None
        self.colorized_image = None
        self.blurred_image = None
        self.image_mask = None"""

    def __init__(self, image_data=None, image_filepath=None, image_size_x=0, image_size_y=0):
        self.image_array = None
        self.image_color = None
        self.image_title = None
        self.blur_kernel = None
        self.mask_lower_limit = None
        self.mask_upper_limit = None
        self.image_size_x = image_size_x
        self.image_size_y = image_size_y

        # If the image is only given as a filepath, read the image from the file
        if image_filepath is not None and image_data is None:
            # self.image_filepath = os.path.join(os.path.dirname(__file__), image_filepath)
            # assert os.path.exists(self.image_filepath)
            self.image_filepath = image_filepath
            self.original_image = cv2.imread(image_filepath)
            self.image_size_x = self.original_image.shape[1]
            self.image_size_y = self.original_image.shape[0]
            # print(self.original_image)

        # Else if the image is given as an array, unpack the array into a multidimensional object
        elif image_data is not None:
            self.image_filepath = None
            """if len(image_data.shape) < 3:
                three_channel_image_data = cv2.cvtColor(image_data, cv2.COLOR_GRAY2BGR)
                """
            """three_channel_image_data = zeros((image_data.shape[0], image_data.shape[1], 3))
                for row_index in range(three_channel_image_data.shape[0]):
                    for column_index in range(three_channel_image_data.shape[1]):
                        for channel_index in range(three_channel_image_data.shape[2]):
                            three_channel_image_data[row_index][channel_index][channel_index] = three_channel_image_data[row_index][column_index][channel_index] + image_data[row_index][column_index]"""
            """
            else:
                three_channel_image_data = image_data
            self.original_image = three_channel_image_data  # self.unpack_single_dimensional_array(image_data)"""
            self.original_image = image_data
            self.image_size_x = self.original_image.shape[1]
            self.image_size_y = self.original_image.shape[0]

        # Otherwise initialize an empty object
        else:
            self.image_filepath = None
            self.original_image = None

        self.cropped_image = None
        self.colorized_image = None
        self.blurred_image = None
        self.image_mask = None
        self.expanded_image_mask = None
        self.contours_in_image = None
        self.contour_centroids = []
        self.image_figure = None

    def unpack_single_dimensional_array(self, data):
        return data

    def set_blur_kernel(self, new_kernel):
        self.blur_kernel = new_kernel

    def set_mask_limits(self, lower_limit, upper_limit):
        self.mask_lower_limit = lower_limit
        self.mask_upper_limit = upper_limit

    def generate_contours_in_image(self):
        self.contours_in_image = []
        if self.image_mask is not None and not sum(self.image_mask) == 0:
            contours_in_image, hierarchy = cv2.findContours(
                self.image_mask.astype(uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_NONE
            )
            temp_contours = sorted(contours_in_image,
                                   key=cv2.contourArea,
                                   reverse=True)
            for contour in temp_contours:
                if cv2.contourArea(contour) > 10:
                    self.contours_in_image.append(contour)

    def calculate_image_centroids(self):

        # create empty array to return when no centroids are found
        result = []

        # calculate the centroid of each contour
        for contour in self.contours_in_image:
            temp_moments = cv2.moments(contour)
            temp_centroid_x = int(temp_moments["m10"] / temp_moments["m00"])
            temp_centroid_y = int(temp_moments["m01"] / temp_moments["m00"])
            self.contour_centroids.append((temp_centroid_x, temp_centroid_y))

        # append the centroid of the largest contour found
        result.append((self.contours_in_image[0], self.contour_centroids[0]))

        # if 2 or more contours were found and the second contour also big, include it
        if len(self.contour_centroids) >= 2:
            if (cv2.contourArea(self.contours_in_image[1]) >=
                    0.9 * cv2.contourArea(self.contours_in_image[0])):
                result.append((self.contours_in_image[1], self.contour_centroids[1]))

        return result

    def get_original_image(self):
        return self.original_image

    def get_colorized_image(self):
        return self.colorized_image

    def set_colorized_image(self, colorized_image):
        self.colorized_image = colorized_image

    def get_blurred_image(self):
        return self.blurred_image

    def set_blurred_image(self, blurred_image):
        self.blurred_image = blurred_image

    def get_image_mask(self):
        return self.image_mask

    def set_image_mask(self, image_mask):
        self.image_mask = image_mask

    def generate_mask_overlaid_image(self):
        pass

    def plot_images(self, figure=None):
        if figure is None and self.image_figure is None:
            figure_dimensions = (4, 12)
            figure = plt.figure(figsize=figure_dimensions)
        if self.image_figure is None:
            self.image_figure = figure

        num_image_rows = 5
        num_image_cols = 1
        row_index = 1

        for image in [self.original_image,
                      self.colorized_image,
                      self.blurred_image,
                      self.image_mask
                      ]:
            if image is not None:
                temp_image_axis = self.image_figure.add_subplot(
                    num_image_rows,
                    num_image_cols,
                    row_index
                )
                temp_image_axis.set_title("")
                temp_image_axis.set_xticks([])
                temp_image_axis.set_yticks([])
                plt.imshow(image, cmap='gray')
                row_index = row_index + 1
        self.image_figure.tight_layout()
