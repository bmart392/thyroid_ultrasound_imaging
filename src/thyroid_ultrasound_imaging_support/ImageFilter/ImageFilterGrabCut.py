"""
Define object class for GrabCut based image filters.
"""

# Import standard python packages
from numpy import uint8, where
from cv2 import GC_FGD, GC_PR_BGD, GC_BGD, GC_PR_FGD

# Import the super-class for all image filters
from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilter import *


class ImageFilterGrabCut(ImageFilter):
    """
    A class for filter objects that use the GrabCut segmentation technique.
    """

    def __init__(self, previous_mask_array: np.array = None, image_crop_coordinates: iter = None,
                 image_crop_included: bool = False, include_pre_blurring: bool = False,
                 increase_contrast=False, down_sampling_rate: float = 0.4,  # 0.5
                 segmentation_iteration_count: int = 1,  # 1
                 sure_foreground_creation_iterations: int = 3,
                 sure_background_creation_iterations: int = 8,
                 debug_mode: bool = False, analysis_mode: bool = False):

        """
        Initialization function for Grabcut Filters.

        Parameters
        ----------
        previous_mask_array
            an array created by the user identifying the background, probable background,
            and the foreground of the image.

        image_crop_coordinates
            an iterable containing two points defining the rectangle with which to crop the image.

        image_crop_included
            a flag to indicate if the image needs to be cropped.

        include_pre_blurring
            a flag to indicate that the image should be filtered prior to segmentation.

        segmentation_iteration_count
            the number of iterations that the grabcut filter runs on each image.

        debug_mode
            a flag indicating that helpful information should be displayed in the terminal during runtime.

        analysis_mode
            a flag indicating that the length of time required to complete each step should be calculated
            and displayed to the user.
        """

        # Call to super class for proper class extension
        super(ImageFilterGrabCut, self).__init__()

        # Update the parameters passed in on object creation
        self.image_crop_included = image_crop_included
        self.image_crop_coordinates = image_crop_coordinates
        self.include_pre_blurring = include_pre_blurring
        self.debug_mode = debug_mode
        self.analysis_mode = analysis_mode
        self.increase_contrast = increase_contrast
        self.down_sampling_rate = down_sampling_rate
        self.up_sampling_rate = 1 / down_sampling_rate
        self.segmentation_iteration_count = segmentation_iteration_count
        self.sure_foreground_creation_iterations = sure_foreground_creation_iterations
        self.sure_background_creation_iterations = sure_background_creation_iterations

        # Define characteristics of all GrabCut filters
        self.filter_color = COLOR_RGB
        self.previous_image_mask_array = previous_mask_array

        # Define a flag to determine when NOT to create a previous-image mask from the current mask
        self.do_not_create_new_previous_image_mask = False

    def filter_image(self, image_data: ImageData):

        try:
            # Perform the basic image filtering actions
            self.basic_filter_image(image_data)

        # Otherwise raise another exception with the root cause
        except SegmentationError as caught_exception:
            raise SegmentationError(FILTER_IMAGE_FAILURE + " in 'filter_image' in ImageFilterGrabCut.py",
                                    caught_exception.history)

        # Note the current time
        start_of_process_time = time()

        # Create mask for segmentation of next image
        try:
            self.create_previous_image_mask(image_data)

        # Otherwise raise an error
        except Exception as caught_exception:
            raise SegmentationError(CREATE_PREVIOUS_IMAGE_MASK_FAILURE + " in 'filter_image' in ImageFilter.py",
                                    caught_exception.args[0])

        self.display_process_timer(start_of_process_time, "Previous Image Mask Creation")

    def pre_process_image(self, image_data: ImageData):
        """
        Increase contrast and blur, if included, the given ImageData object. Overrides the super-class definition.

        Parameters
        ----------
        image_data
            the ImageData object containing the image to be pre-processed.
        """
        # Create a temporary copy of the image
        temp_image = copy(image_data.down_sampled_image)

        # Increase the contrast of the image, if required
        if self.increase_contrast:
            temp_image = cv2.equalizeHist(temp_image)

        # If the filter includes a blurring affect
        if self.include_pre_blurring:
            """
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
                                                                                  """

        else:

            # Set the pre-processed image as a copy of the colorized image.
            image_data.pre_processed_image = temp_image

    def create_image_mask(self, image_data: ImageData):
        """
        Segment the image using the GrabCut method. Overrides the super-class definition.

        Parameters
        ----------
        image_data
            the ImageData object containing the image to be segmented.
        """

        # Save the size of the image
        image_size = image_data.pre_processed_image.shape

        # Define the initialized rectangle of the image for the GrabCut algorithm
        initialized_rectangle_cords = (0, 0, image_size[0], image_size[1])

        # Set the mask to use to segment the image as a copy of the previous image mask.
        # Check that the previous_image_mask has been initialized and raise an error if it has not.
        if self.previous_image_mask_array is None:
            raise Exception("Previous image mask array was not created.")
        image_data.segmentation_initialization_mask = copy(self.previous_image_mask_array)

        # Segment the image
        cv2.grabCut(img=image_data.pre_processed_image,
                    mask=image_data.segmentation_initialization_mask,
                    rect=initialized_rectangle_cords,
                    bgdModel=np.zeros((1, 65), np.float64),
                    fgdModel=np.zeros((1, 65), np.float64),
                    iterCount=self.segmentation_iteration_count,  # 15
                    mode=cv2.GC_INIT_WITH_MASK
                    )

        # Modify the image mask to a binary result
        image_data.image_mask = np.where(
            (image_data.segmentation_initialization_mask == cv2.GC_PR_BGD) |
            (image_data.segmentation_initialization_mask == cv2.GC_BGD), 0, 1).astype('uint8')

    def post_process_image_mask(self, image_data: ImageData):
        """
        Do nothing to the image mask. Overrides the super-class definition.

        Parameters
        ----------
        image_data
            the ImageData object containing the image mask.
        """
        """if self.down_sampling_rate is not None:
            image_data.image_mask = cv2.resize(image_data.image_mask, (0, 0),
                                               fx=self.down_sampling_rate,
                                               fy=self.down_sampling_rate,
                                               interpolation=cv2.INTER_CUBIC)"""
        image_data.post_processed_mask = image_data.image_mask

    def create_sure_foreground_mask(self, image_data: ImageData):
        """
        Create a mask representing the area of the image that is surely the foreground by
        shrinking the segmentation result.

        Parameters
        ----------
        image_data
            the ImageData object containing the image mask.
        """
        image_data.sure_foreground_mask = cv2.morphologyEx(
            image_data.post_processed_mask, cv2.MORPH_ERODE,
            np.ones((3, 3), np.uint8), iterations=self.sure_foreground_creation_iterations,  # 4 # 10, # 2
            anchor=(1, 1)
        )

    def create_sure_background_mask(self, image_data: ImageData):
        """
        Create a mask representing the area of the image that is surely the background by
        expanding the segmentation result and inverting the mask.

        Parameters
        ----------
        image_data
            the ImageData object containing the image mask.
        """
        image_data.sure_background_mask = 1 - cv2.morphologyEx(
            image_data.post_processed_mask, cv2.MORPH_DILATE,
            np.ones((3, 3), np.uint8), iterations=self.sure_background_creation_iterations,  # 6 # 10
            anchor=(1, 1)
        )

    @staticmethod
    def create_probable_foreground_mask(image_data: ImageData):
        """
        Create a mask representing the area of the image that is probably the foreground by
        including the area that is not in the sure foreground or sure background.

        Parameters
        ----------
        image_data
            the ImageData object containing the image mask.
        """
        image_data.probable_foreground_mask = 1 - (
                image_data.sure_foreground_mask + image_data.sure_background_mask
        )

    def create_previous_image_mask(self, image_data: ImageData):
        """
        Creates the image mask to use to segment the next image.

        Parameters
        ----------
        image_data
            The image data object containing the masks generated from the current image.
        """

        # Do no create a new previous-image-mask if the flag is set as True
        if not self.do_not_create_new_previous_image_mask:
            # Create the image mask to use for the next iteration
            self.previous_image_mask_array = uint8(image_data.sure_foreground_mask * cv2.GC_FGD +
                                                   image_data.sure_background_mask * cv2.GC_BGD +
                                                   image_data.probable_foreground_mask * cv2.GC_PR_FGD)

        # Otherwise, reset the flag
        else:
            self.do_not_create_new_previous_image_mask = False

        """# Crop the image mask if necessary
        if self.image_crop_included:
            crop_x_0 = self.image_crop_coordinates[0][0]
            crop_y_0 = self.image_crop_coordinates[0][1]
            crop_x_1 = self.image_crop_coordinates[1][0]
            crop_y_1 = self.image_crop_coordinates[1][1]
            image_mask_for_next_image = image_mask_for_next_image[crop_y_0:crop_y_1, crop_x_0:crop_x_1]

        # Shrink the image mask if necessary
        self.update_previous_image_mask(image_mask_for_next_image)"""

    def update_previous_image_mask(self, new_previous_image_mask: np.array):
        """
        Updates the previous image mask array of the filter by down-sampling the given mask.

        Parameters
        ----------
        new_previous_image_mask
            the array containing the mask to use to generate the new previous image mask.
        """
        if self.down_sampling_rate is not None:

            # Separate each area of the mask
            fg_mask = uint8(where(new_previous_image_mask == GC_FGD, GC_FGD, 0))
            pr_fgd_mask = uint8(where(new_previous_image_mask == GC_PR_FGD, GC_PR_FGD, 0))

            # Resize each area as its own mask
            fg_mask = cv2.resize(fg_mask, RESIZE_SHAPE,
                                 fx=self.down_sampling_rate,
                                 fy=self.down_sampling_rate,
                                 interpolation=DOWN_SAMPLING_MODE)
            pr_fgd_mask = cv2.resize(pr_fgd_mask, RESIZE_SHAPE,
                                     fx=self.down_sampling_rate,
                                     fy=self.down_sampling_rate,
                                     interpolation=DOWN_SAMPLING_MODE)

            # Remove any artifacts leftover from the resizing process
            pr_fgd_mask = uint8(where(pr_fgd_mask != GC_BGD, GC_PR_FGD, 0))

            # Combine them back together
            combined_mask = fg_mask + pr_fgd_mask

            # Separate each area of the mask
            fg_mask = uint8(where(combined_mask == GC_FGD, GC_FGD, 0))
            pr_fgd_mask = uint8(where(combined_mask == GC_PR_FGD, GC_PR_FGD, 0))
            error_adjustment_mask = uint8(where(combined_mask == GC_FGD + GC_PR_FGD, GC_PR_FGD, 0))

            # Combine the masks back together to get the final previous-image mask
            self.previous_image_mask_array = fg_mask + pr_fgd_mask + error_adjustment_mask

        else:
            self.previous_image_mask_array = new_previous_image_mask
