"""
Define object class for GrabCut based image filters.
"""
import cv2
import matplotlib.pyplot as plt
import numpy as np

# Import constants and image statements used in all filter objects
from scripts.Filters.FilterHelper import *


class ImageFilterGrabCut:

    def __init__(self, user_created_mask_array: np.array,
                 image_crop_coordinates: np.array, image_crop_included: bool = False,
                 debug_mode: bool = False, analysis_mode: bool = False):
        self.filter_color = COLOR_BGR
        self.use_previous_image_mask = False
        self.previous_image_mask_array = None
        self.user_created_mask_array = user_created_mask_array

        self.image_crop_included = image_crop_included
        self.image_crop_coordinates = image_crop_coordinates

        self.debug_mode = debug_mode
        self.analysis_mode = analysis_mode

    def fully_filter_image(self, image_data: ImageData, previous_image_mask_array: np.array = None) -> ImageData:
        if previous_image_mask_array is not None:
            self.previous_image_mask_array = previous_image_mask_array
        return fully_filter_image(self, image_data)

    @staticmethod
    def pre_process_image(image_data: ImageData) -> ImageData:
        if True:
            sigma_est = np.mean(estimate_sigma(image_data.colorized_image, channel_axis=2))
            patch_kw = dict(patch_size=10,
                            patch_distance=2,
                            channel_axis=2)
            image_data.pre_processed_image = np.uint8(255 * copy(denoise_nl_means(image_data.colorized_image,
                                                                   h=0.6 * sigma_est,
                                                                   sigma=sigma_est,
                                                                   fast_mode=True,
                                                                   **patch_kw)))
        else:
            image_data.pre_processed_image = copy(image_data.colorized_image)
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

    @staticmethod
    def image_mask_post_process(image_data: ImageData) -> ImageData:
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
            np.ones(10, np.uint8), iterations=15
        )
        return image_data

    @staticmethod
    def create_probable_foreground_mask(image_data: ImageData) -> ImageData:
        image_data.probable_foreground_mask = 1 - (
            image_data.sure_foreground_mask + image_data.sure_background_mask
        )
        return image_data
