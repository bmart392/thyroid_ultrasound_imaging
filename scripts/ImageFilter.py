import cv2
import matplotlib.pyplot as plt
import numpy as np
from ImageData import *
import FilterConstants as FC
from time import time
from display_processor_time import display_process_timer


class ImageFilter:
    """
    A class for defining image filters for images in the GRAY color scheme using standard processes.
    """

    def __init__(self, filter_color=FC.ColorOptions.GRAY,
                 blur_type=FC.BlurTypes.GAUSSIAN,
                 blur_parameters=None,
                 thresholding_type=FC.ThresholdOptions.BASIC,
                 thresholding_parameters=None,
                 mask_manipulation_sequence=None,
                 filter_name=None,
                 debug_mode=False,
                 analysis_mode=False):
        """
        Create a class for defining image filters for images in the GRAY color scheme using standard processes.

        Parameters
        ----------
        filter_color: int
            select final color scheme of the image. Defaults to grayscale.

        blur_type: int
            select the type of blurring used on the image. Defaults to MEAN_FILTER.

        blur_parameters: tuple
            select the parameters of the blurring. Defaults to (55, 55).

        thresholding_type: int
            select the type of thresholding used on the image. Defaults to BASIC.

        thresholding_parameters: tuple
            select the upper and lower bounds of the thresholding used on the image.
            Defaults to (65, 110).

        mask_manipulation_sequence: list
            sequence of mask manipulation steps as a list of tuples with the format of
            (manipulation_type, parameters, num_iterations) where manipulation_type is
            defined in FilterConstants file.
            Defaults to CLOSE then ERODE.

        debug_mode: bool
            display graphics and additional print statements helpful in the debugging process.

        analysis_mode: bool
            display the time key processes take to occur.
        """

        # Filter object parameters
        self.filter_name = filter_name
        self.filter_color = filter_color

        # Blur parameters
        self.blur_type = blur_type
        if blur_parameters is None:
            blur_parameters = (5, 5, 5)  # (13, 13, 2.5) (45, 45)
        self.blur_parameters = blur_parameters

        # Thresholding parameters
        self.thresholding_type = thresholding_type
        if thresholding_parameters is None:
            thresholding_parameters = (55, 100)
        self.thresholding_parameters = thresholding_parameters

        # Mask manipulation parameters
        if mask_manipulation_sequence is None:
            # mask_manipulation_sequence = []
            mask_manipulation_sequence = [
                # (FC.MaskManipulationOptions.RECT, (4, 4), 2),
                (FC.MaskManipulationOptions.CLOSE, (10, 10), 1),  # (10, 10, 1)
                (FC.MaskManipulationOptions.ERODE, (6, 6), 2),  # (10, 10, 1)
                # (FC.MaskManipulationOptions.OPEN, (5, 5), 1),
            ]
        self.mask_manipulation_sequence = mask_manipulation_sequence
        self.image_cutoff = np.concatenate((np.zeros((140, 640)), np.ones((340, 640))))
        self.debug_mode = debug_mode
        self.analysis_mode = analysis_mode

    def colorize_image(self, image_data: ImageData) -> ImageData:
        """
        Recolor the original image based on the parameters of the image filter.

        Parameters
        ----------
        image_data: ImageData
            the image data object containing the image to be recolored.
        """

        # select the recoloring color if the filter_color is not GRAY
        if not self.filter_color == FC.ColorOptions.GRAY:

            if self.filter_color == FC.ColorOptions.BGR:
                color = cv2.COLOR_GRAY2BGR
            elif self.filter_color == FC.ColorOptions.RGB:
                color = cv2.COLOR_GRAY2RGB
            else:
                color = cv2.COLOR_GRAY2BGR

            # set the colorized image
            image_data.colorized_image = cv2.cvtColor(image_data.get_original_image(), color)

        # otherwise set the colorized image as the original image
        else:
            image_data.colorized_image = image_data.original_image

        # return the image data object
        return image_data

    def blur_image(self, image_data: ImageData):
        """
        Blur the image based on the parameters of the image filter using the original image.

        Parameters
        ----------
        image_data: ImageData
            the image data object containing the image to be blurred.
        """
        need_to_recolor = False
        color = cv2.COLOR_BGR2GRAY

        if self.filter_color == FC.ColorOptions.RGB:
            need_to_recolor = True
            color = cv2.COLOR_BGR2RGB
        elif self.filter_color == FC.ColorOptions.GRAY:
            need_to_recolor = True
            color = cv2.COLOR_BGR2GRAY

        if self.blur_type == FC.BlurTypes.GAUSSIAN:
            image_data.set_blurred_image(
                cv2.GaussianBlur(image_data.get_colorized_image(),
                                 (self.blur_parameters[0], self.blur_parameters[1]),
                                 self.blur_parameters[2]
                                 )
            )
            """if need_to_recolor:
                image_data.set_blurred_image(
                    cv2.cvtColor(image_data.get_blurred_image(),
                                 color)
                )"""
            return image_data
        elif self.blur_type == FC.BlurTypes.MEAN_FILTER:
            image_data.set_blurred_image(
                cv2.pyrMeanShiftFiltering(image_data.get_original_image(),
                                          self.blur_parameters[0],
                                          self.blur_parameters[1]
                                          )
            )
            if need_to_recolor:
                image_data.set_blurred_image(
                    cv2.cvtColor(image_data.get_blurred_image(),
                                 color)
                )
            return image_data
        else:
            image_data.set_blurred_image(image_data.get_colorized_image())
            return image_data

    def create_image_mask(self, image_data: ImageData):
        if self.thresholding_type == FC.ThresholdOptions.BASIC:
            image_data.set_image_mask(
                cv2.inRange(image_data.get_blurred_image(),
                            self.thresholding_parameters[0],
                            self.thresholding_parameters[1]
                            )
            )
            image_data.set_image_mask(image_data.image_mask * self.image_cutoff)
            return image_data
        elif self.thresholding_type == FC.ThresholdOptions.ADAPTIVE:
            threshold_values, image_data.image_mask = cv2.adaptiveThreshold(image_data.get_blurred_image(),
                                                                            1, cv2.ADAPTIVE_THRESH_MEAN_C,
                                                                            cv2.THRESH_BINARY,
                                                                            75, 1
                                                                            )

            image_data.image_mask = image_data.image_mask * self.image_cutoff
            return image_data

    def modify_image_mask(self, image_data: ImageData):
        for modification in self.mask_manipulation_sequence:
            modification_type = modification[0]
            if modification_type == FC.MaskManipulationOptions.RECT:
                modification_type = cv2.MORPH_RECT
            elif modification_type == FC.MaskManipulationOptions.CLOSE:
                modification_type = cv2.MORPH_CLOSE
            elif modification_type == FC.MaskManipulationOptions.OPEN:
                modification_type = cv2.MORPH_OPEN
            elif modification_type == FC.MaskManipulationOptions.ERODE:
                modification_type = cv2.MORPH_ERODE
            image_data.set_image_mask(
                cv2.morphologyEx(image_data.get_image_mask(),
                                 modification_type,
                                 np.ones(modification[1], np.uint8),
                                 iterations=modification[2]
                                 )
            )
        return image_data

    def fully_filter_image(self, image_data: ImageData):

        # record the current time for timing each process
        start_of_process_time = time()

        # Recolor the image
        image_data = self.colorize_image(image_data)
        if self.debug_mode:
            cv2.imshow("recolored image", image_data.colorized_image)
            cv2.waitKey(1)
        start_of_process_time = display_process_timer(start_of_process_time, "Recolor image time", self.analysis_mode)

        # Blur the image
        image_data = self.blur_image(image_data)
        if self.debug_mode:
            cv2.imshow("blurred image", image_data.blurred_image)
            cv2.waitKey(1)
        start_of_process_time = display_process_timer(start_of_process_time, "Blur image time", self.analysis_mode)

        # Segment the image
        image_data = self.create_image_mask(image_data)
        image_data = self.modify_image_mask(image_data)
        if self.debug_mode:
            cv2.imshow("image mask", image_data.image_mask)
            cv2.waitKey(1)
        start_of_process_time = display_process_timer(start_of_process_time, "Image mask creation time",
                                                      self.analysis_mode)

        # Find the contours in the image
        image_data.generate_contours_in_image()
        start_of_process_time = display_process_timer(start_of_process_time, "Contour generation time", self.analysis_mode)

        # If there are contours in the image, find their centroids
        if len(image_data.contours_in_image) > 0:
            image_data.calculate_image_centroids()
        display_process_timer(start_of_process_time, "Centroid calculation", self.analysis_mode)

        return image_data


if __name__ == '__main__':
    # Create test data using example picture from computer
    test_image_data = ImageData(image_data=None, image_filepath='/home/ben/Pictures/thyroid_ultrasound.png')

    # Create a temporary image filter
    image_filter = ImageFilter()

    # Filter the image
    test_image_data = image_filter.fully_filter_image(test_image_data)

    # Show the filtered images
    test_image_data.plot_images()

    print(test_image_data.contour_centroids)

    print("hi")
