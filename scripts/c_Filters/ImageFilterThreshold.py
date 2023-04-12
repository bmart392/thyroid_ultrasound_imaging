"""
Define object class for threshold-based image filters.
"""
# Import constants and image statements used in all filter objects
from scripts.c_Filters.ImageFilter import *


class ImageFilterThreshold(ImageFilter):
    """
    A class for defining image filters for images in the GRAY color scheme using standard processes.
    """

    def __init__(self, filter_color=COLOR_GRAY,
                 blur_type=BLUR_GAUSSIAN,
                 blur_parameters=None,
                 thresholding_type=THRESHOLD_BASIC,
                 thresholding_parameters=None,
                 mask_manipulation_sequence=None,
                 filter_name=None,
                 debug_mode=False,
                 analysis_mode=False,
                 image_crop_included=False,
                 image_crop_coordinates=None,
                 increase_contrast=False):

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
        super(ImageFilterThreshold, self).__init__()
        # Filter object parameters
        self.filter_name = filter_name
        self.filter_color = filter_color

        self.increase_contrast = increase_contrast

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
                # (MASK_RECT, (4, 4), 2),
                # (MASK_CLOSE, (10, 10), 1),  # (10, 10, 1)
                # (MASK_ERODE, (6, 6), 2),  # (10, 10, 1)
                # (MASK_OPEN, (5, 5), 1),
            ]
        self.mask_manipulation_sequence = mask_manipulation_sequence
        self.image_cutoff = np.concatenate((np.zeros((140, 640)), np.ones((340, 640))))
        self.debug_mode = debug_mode
        self.analysis_mode = analysis_mode

        self.image_crop_included = image_crop_included
        if image_crop_coordinates is None:
            self.image_crop_coordinates = [[0, 0], [0, 0]]
        else:
            self.image_crop_coordinates = image_crop_coordinates

    def pre_process_image(self, image_data: ImageData):
        """
        Blur the image based on the parameters of the image filter using the original image.

        Parameters
        ----------
        image_data: ImageData
            the image data object containing the image to be blurred.
        """
        need_to_recolor = False
        color = cv2.COLOR_BGR2GRAY

        temp_image = copy(image_data.colorized_image)

        if self.increase_contrast:
            temp_image = cv2.equalizeHist(temp_image)

        if self.filter_color == COLOR_RGB:
            need_to_recolor = True
            color = cv2.COLOR_BGR2RGB
        elif self.filter_color == COLOR_GRAY:
            need_to_recolor = True
            color = cv2.COLOR_BGR2GRAY

        if self.blur_type == BLUR_GAUSSIAN:
            image_data.pre_processed_image = (
                cv2.GaussianBlur(temp_image,
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
        elif self.blur_type == BLUR_MEAN_FILTER:
            image_data.pre_processed_image = (
                cv2.pyrMeanShiftFiltering(temp_image,
                                          self.blur_parameters[0],
                                          self.blur_parameters[1]
                                          )
            )
            if need_to_recolor:
                image_data.pre_processed_image = cv2.cvtColor(image_data.pre_processed_image,
                                                              color)

            return image_data
        else:
            image_data.pre_processed_image = image_data.colorized_image
            return image_data

    def create_image_mask(self, image_data: ImageData):
        """
        Segment the image using a thresholding method. Overrides the super-class definition.

        Parameters
        ----------
        image_data
            the ImageData object containing the image to be segmented.
        """
        if self.thresholding_type == THRESHOLD_BASIC:
            image_data.image_mask = cv2.inRange(image_data.pre_processed_image,
                                                self.thresholding_parameters[0],
                                                self.thresholding_parameters[1]
                                                )

            image_data.image_mask = np.uint8(image_data.image_mask / 255)
            return image_data
        elif self.thresholding_type == THRESHOLD_ADAPTIVE:
            threshold_values, image_data.image_mask = cv2.adaptiveThreshold(image_data.pre_processed_image,
                                                                            1, cv2.ADAPTIVE_THRESH_MEAN_C,
                                                                            cv2.THRESH_BINARY,
                                                                            75, 1
                                                                            )

            image_data.image_mask = np.uint8(image_data.image_mask / 255)

    def image_mask_post_process(self, image_data: ImageData):
        """
        Conduct morphological operations on the final mask to improve the segmentation result.
        Overrides the super-class definition.

        Parameters
        ----------
        image_data
            the ImageData object containing the image.
        """

        # Conduct the segmentation modifications listed in the filter
        for modification in self.mask_manipulation_sequence:
            modification_type = modification[0]
            if modification_type == MASK_RECT:
                modification_type = cv2.MORPH_RECT
            elif modification_type == MASK_CLOSE:
                modification_type = cv2.MORPH_CLOSE
            elif modification_type == MASK_OPEN:
                modification_type = cv2.MORPH_OPEN
            elif modification_type == MASK_ERODE:
                modification_type = cv2.MORPH_ERODE
            image_data.image_mask = cv2.morphologyEx(image_data.image_mask,
                                                     modification_type,
                                                     np.ones(modification[1], np.uint8),
                                                     iterations=modification[2]
                                                     )

    @staticmethod
    def create_sure_foreground_mask(image_data: ImageData):
        # TODO - Implement this function properly
        image_data.sure_foreground_mask = np.zeros(image_data.expanded_image_mask.shape[:2], np.uint8)

    @staticmethod
    def create_sure_background_mask(image_data: ImageData):
        # TODO - Implement this function properly
        image_data.sure_background_mask = np.zeros(image_data.expanded_image_mask.shape[:2], np.uint8)

    @staticmethod
    def create_probable_foreground_mask(image_data: ImageData):
        # TODO - Implement this function properly
        image_data.probable_foreground_mask = np.ones(image_data.expanded_image_mask.shape[:2], np.uint8)


