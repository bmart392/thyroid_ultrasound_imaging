"""
Define object class for GrabCut based image filters.
"""

# Import constants and image statements used in all filter objects
from scripts.Filters.FilterHelper import *

# Import the super-class for all image filters
from scripts.Filters.ImageFilter import ImageFilter


class ImageFilterGrabCut(ImageFilter):
    """
    A class for filter objects that use the GrabCut segmentation technique.
    """

    def __init__(self, user_created_mask_array: np.array, image_crop_coordinates: iter,
                 image_crop_included: bool = False, include_pre_blurring: bool = False,
                 increase_contrast = False,
                 debug_mode: bool = False, analysis_mode: bool = False):

        """
        Initialization function for Grabcut Filters.

        Parameters
        ----------
        user_created_mask_array
            an array created by the user identifying the background, probable background,
            and the foreground of the image.

        image_crop_coordinates
            an iterable containing two points defining the rectangle with which to crop the image.

        image_crop_included
            a flag to indicate if the image needs to be cropped.

        include_pre_blurring
            a flag to indicate that the image should be filtered prior to segmentation.

        debug_mode
            a flag indicating that helpful information should be displayed in the terminal during runtime.

        analysis_mode
            a flag indicating that the length of time required to complete each step should be calculated
            and displayed to the user.
        """

        # Call to super class for proper class extension
        super(ImageFilterGrabCut, self).__init__()

        # Update the parameters passed in on object creation
        self.user_created_mask_array = user_created_mask_array
        self.user_created_mask_array = user_created_mask_array
        self.image_crop_included = image_crop_included
        self.image_crop_coordinates = image_crop_coordinates
        self.include_pre_blurring = include_pre_blurring
        self.debug_mode = debug_mode
        self.analysis_mode = analysis_mode
        self.increase_contrast = increase_contrast

        # Define characteristics of all GrabCut filters
        self.filter_color = COLOR_BGR
        self.use_previous_image_mask = False
        self.previous_image_mask_array = None

    def fully_filter_image(self, image_data: ImageData, previous_image_mask_array: np.array = None) -> ImageData:
        """
        Fully filter the given ImageData using the stored previous_image_mask. If a previous_image_mask is given,
        use that one instead.

        Parameters
        ----------
        image_data
            the ImageData object containing the image to be segmented.
        previous_image_mask_array
            the mask to use to segment the image when one is available.
        """

        if previous_image_mask_array is not None:
            self.previous_image_mask_array = previous_image_mask_array
        return fully_filter_image(self, image_data)

    def pre_process_image(self, image_data: ImageData) -> ImageData:
        """
        Pre-process the given ImageData object.

        Parameters
        ----------
        image_data
            the ImageData object containing the image to be pre-processed.
        """
        temp_image = copy(image_data.colorized_image)
        if self.increase_contrast:
            temp_image = cv2.equalizeHist(temp_image)


        # If the filter includes a blurring affect
        if self.include_pre_blurring:

            # Blur the image
            sigma_est = np.mean(estimate_sigma(image_data.colorized_image, channel_axis=2))
            patch_kw = dict(patch_size=10,
                            patch_distance=2,
                            channel_axis=2)
            image_data.pre_processed_image = np.uint8(255 * copy(denoise_nl_means(temp_image,
                                                                                  h=0.6 * sigma_est,
                                                                                  sigma=sigma_est,
                                                                                  fast_mode=True,
                                                                                  **patch_kw)))
        else:

            # Set the pre-processed image as a copy of the colorized image.
            image_data.pre_processed_image = temp_image

        # Return the image_data object for future use
        return image_data

    def create_image_mask(self, image_data: ImageData) -> ImageData:

        image_size = image_data.pre_processed_image.shape

        initialized_rectangle_cords = (0, 0, image_size[0], image_size[1])
        if self.use_previous_image_mask and self.previous_image_mask_array is not None:
            initialization_mask_to_use = self.previous_image_mask_array
        else:
            initialization_mask_to_use = np.copy(self.user_created_mask_array)

        image_data.segmentation_initialization_mask = copy(initialization_mask_to_use)

        cv2.grabCut(img=image_data.pre_processed_image,
                    mask=initialization_mask_to_use,
                    rect=initialized_rectangle_cords,
                    bgdModel=np.zeros((1, 65), np.float64),
                    fgdModel=np.zeros((1, 65), np.float64),
                    iterCount=15,
                    mode=cv2.GC_INIT_WITH_MASK
                    )

        image_data.image_mask = np.where((initialization_mask_to_use == cv2.GC_PR_BGD) |
                                         (initialization_mask_to_use == cv2.GC_BGD), 0, 1).astype('uint8')

        return image_data

    def image_mask_post_process(self, image_data: ImageData) -> ImageData:
        return image_data

    @staticmethod
    def create_sure_foreground_mask(image_data: ImageData) -> ImageData:
        image_data.sure_foreground_mask = cv2.morphologyEx(
            image_data.expanded_image_mask, cv2.MORPH_ERODE,
            np.ones(10, np.uint8), iterations=2
        )
        return image_data

    @staticmethod
    def create_sure_background_mask(image_data: ImageData) -> ImageData:
        image_data.sure_background_mask = 1 - cv2.morphologyEx(
            image_data.expanded_image_mask, cv2.MORPH_DILATE,
            np.ones(3, np.uint8), iterations=30
        )
        return image_data

    @staticmethod
    def create_probable_foreground_mask(image_data: ImageData) -> ImageData:
        image_data.probable_foreground_mask = 1 - (
                image_data.sure_foreground_mask + image_data.sure_background_mask
        )
        return image_data
