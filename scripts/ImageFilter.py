import matplotlib.pyplot as plt
import numpy as np
from ImageData import *
import FilterConstants as FC


class ImageFilter:

    def __init__(self, filter_color=FC.ColorOptions.GRAY,
                 blur_type=FC.BlurTypes.MEAN_FILTER,
                 blur_parameters=None,
                 thresholding_type=FC.ThresholdOptions.BASIC,
                 thresholding_parameters=None,
                 mask_manipulation_sequence=None,
                 filter_name=None):

        self.filter_name = filter_name
        self.filter_color = filter_color
        self.blur_type = blur_type
        if blur_parameters is None:
            blur_parameters = (45, 45)
        self.blur_parameters = blur_parameters
        self.thresholding_type = thresholding_type
        if thresholding_parameters is None:
            thresholding_parameters = (55, 90)
        self.thresholding_parameters = thresholding_parameters
        if mask_manipulation_sequence is None:
            mask_manipulation_sequence = [(FC.MaskManipulationOptions.RECT, (4, 4), 2),
                                          (FC.MaskManipulationOptions.CLOSE, (4, 4), 3),
                                          (FC.MaskManipulationOptions.ERODE, (4, 4), 6)
                                          ]
        self.mask_manipulation_sequence = mask_manipulation_sequence

    def colorize_image(self, image_data: ImageData):
        if self.filter_color == FC.ColorOptions.BGR:
            image_data.colorized_image = image_data.original_image
            return
        elif self.filter_color == FC.ColorOptions.RGB:
            color = cv2.COLOR_BGR2RGB
        elif self.filter_color == FC.ColorOptions.GRAY:
            color = cv2.COLOR_BGR2GRAY
        image_data.set_colorized_image(cv2.cvtColor(image_data.get_original_image(), color))
        return image_data

    def blur_image(self, image_data: ImageData):
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
            if need_to_recolor:
                image_data.set_blurred_image(
                    cv2.cvtColor(image_data.get_blurred_image(),
                                 color)
                )
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
            return image_data
        elif self.thresholding_type == FC.ThresholdOptions.ADAPTIVE:
            image_data.set_image_mask(
                cv2.adaptiveThreshold(image_data.get_blurred_image(),
                                      1, cv2.ADAPTIVE_THRESH_MEAN_C,
                                      cv2.THRESH_BINARY,
                                      75, 1
                                      )
            )
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
        image_data = self.colorize_image(image_data)
        image_data = self.blur_image(image_data)
        image_data = self.create_image_mask(image_data)
        image_data = self.modify_image_mask(image_data)
        image_data.generate_contours_in_image()
        image_data.calculate_image_centroids()
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

