"""
Define object class for parent of all filter types.
"""

# Import standard package elements
import numpy as np

# Import custom class elements
from scripts.ImageData.ImageData import ImageData


class ImageFilter:
    """
    A parent class for all image filters to extend. Defines basic function calls
    """

    def __init__(self):
        self.filter_color = None
        self.use_previous_image_mask = False
        self.previous_image_mask_array = None
        self.user_created_mask_array = None

    def fully_filter_image(self, image_data: ImageData, previous_image_mask_array: np.array = None) -> ImageData:
        raise Exception("This function was not implemented in the sub-class.")

    def pre_process_image(self, image_data: ImageData) -> ImageData:
        raise Exception("This function was not implemented in the sub-class.")

    def create_image_mask(self, image_data: ImageData) -> ImageData:
        raise Exception("This function was not implemented in the sub-class.")

    def image_mask_post_process(self, image_data: ImageData) -> ImageData:
        raise Exception("This function was not implemented in the sub-class.")
