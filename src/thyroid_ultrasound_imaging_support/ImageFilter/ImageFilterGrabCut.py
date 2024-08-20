"""
Define object class for GrabCut based image filters.
"""

# Import standard python packages
from numpy import uint8, where, median, std, ones, zeros, array
from cv2 import GC_FGD, GC_PR_BGD, GC_BGD, GC_PR_FGD, convertScaleAbs, morphologyEx, MORPH_ERODE, MORPH_DILATE
from copy import deepcopy

# Import the super-class for all image filters
from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilter import *
from thyroid_ultrasound_imaging_support.Validation.create_dice_score_mask import create_dice_score_mask
from thyroid_ultrasound_support.Constants.SharedConstants import GROWTH_PHASE, REST_PHASE


class ImageFilterGrabCut(ImageFilter):
    """
    A class for filter objects that use the GrabCut segmentation technique.
    """

    def __init__(self, previous_mask_array: np.array = None, image_crop_coordinates: iter = None,
                 include_pre_blurring: bool = False,
                 increase_contrast=True, down_sampling_rate: float = 0.30,  # 0.35, 0.5
                 segmentation_iteration_count: int = 1,  # 1
                 sure_foreground_creation_iterations: int = None,  # 3
                 sure_background_creation_iterations: int = None,  # 8
                 sure_foreground_creation_dice_score: float = 0.8,
                 sure_background_creation_dice_score: float = 1.25,
                 sure_foreground_creation_kernel_size: float = (3, 3),
                 sure_background_creation_kernel_size: float = (5, 5),
                 segmentation_phase: str = GROWTH_PHASE,
                 bright_threshold_upper_bound: float = 2.0,
                 bap_cutoff_percentage: float = 1.25, map_cutoff_percentage: float = 9.,
                 num_previous_masks_to_combine: float = 20, composite_mask_cutoff_value: float = 0.85,
                 allowed_variation_in_stability: float = 4.,
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

        include_pre_blurring
            a flag to indicate that the image should be filtered prior to segmentation.

        segmentation_iteration_count
            the number of iterations that the grabcut filter runs on each image.

        segmentation_phase
            defines the level of aggression of the filter in expanding and contracting

        bright_threshold_upper_bound
            defines the number of standard deviations above the median value in which a pixel is considered bright
            within the region of interest

        bap_cutoff_percentage
            the percentage of area that the bright area makes up within the region of interest when it may
            start being excluded

        map_cutoff_percentage
            the percentage of area that the region of interest makes up within the whole image when the bright area
            may start being excluded

        num_previous_masks_to_combine
            the number of previous masks to retain for creating the composite previous mask image

        composite_mask_cutoff_value
            the minimum value at which a pixel in the composite mask will be influence the final mask

        allowed_variation_in_stability
            the allowed variation for the segmentation to be considered stable measured as percent of the mean number
             of pixels in each of the previous image masks

        debug_mode
            a flag indicating that helpful information should be displayed in the terminal during runtime.

        analysis_mode
            a flag indicating that the length of time required to complete each step should be calculated
            and displayed to the user.
        """

        # Call to super class for proper class extension
        super(ImageFilterGrabCut, self).__init__()

        # Update the parameters passed in on object creation
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
        self.sure_foreground_creation_dice_score = sure_foreground_creation_dice_score
        self.sure_background_creation_dice_score = sure_background_creation_dice_score
        self.sure_foreground_creation_kernel_size = sure_foreground_creation_kernel_size
        self.sure_background_creation_kernel_size = sure_background_creation_kernel_size

        # Define characteristics of all GrabCut filters
        self.filter_color = COLOR_RGB
        self.previous_image_mask_array = previous_mask_array

        # Define a flag to determine when NOT to create a previous-image mask from the current mask
        self.do_not_create_new_previous_image_mask = False

        # Define variables for the bright area exclusion
        self.segmentation_phase: str = segmentation_phase
        self.bright_threshold_upper_bound = bright_threshold_upper_bound  # Measured in standard deviations
        self.bap_cutoff_percentage = bap_cutoff_percentage
        self.map_cutoff_percentage = map_cutoff_percentage
        self.num_previous_masks_to_combine = num_previous_masks_to_combine
        self.composite_mask_cutoff_value = composite_mask_cutoff_value
        self.previous_image_masks = []

        # Define the allowed level of variation for the segmentation to be considered stable
        self.allowed_variation_in_stability = allowed_variation_in_stability / 100

    def filter_image(self, image_data: ImageData, override_existing_data: bool = False):
        """
        Filters an ImageData object using the process defined in the ImageFilter.

        Parameters
        ----------
        image_data
            the ImageData object containing the image to be filtered.
        override_existing_data
            if true, the segmentation process ignores any previous data and
            starts segmentation process at original image.
        """

        try:
            # Perform the basic image filtering actions
            self.basic_filter_image(image_data, override_existing_data=override_existing_data)

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
            temp_image = cv2.convertScaleAbs(temp_image, alpha=1.5)

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
        Prepare the mask for use to generate the previous image mask

        Parameters
        ----------
        image_data
            the ImageData object containing the image mask.
        """

        if sum(sum(image_data.image_mask)) > 0:

            # diagnostic_data = [('Original Mask', deepcopy(image_data.image_mask))]

            # Capture only the portion of the image that contains the ROI
            region_of_interest = image_data.down_sampled_image[:, :, 0] * image_data.image_mask

            # Capture all the non-zero values within the ROI
            non_zero_values = region_of_interest[region_of_interest > 0]

            # Calculate the median and standard deviation of the intensity value within the ROI
            median_intensity = median(non_zero_values)
            std_dev_intensity = std(non_zero_values)

            # Isolate only the areas that are bright
            bright_area_mask = (median_intensity + (self.bright_threshold_upper_bound * std_dev_intensity) <
                                region_of_interest).astype(uint8)

            # diagnostic_data.append(('Original Bright', deepcopy(bright_area_mask)))

            # Calculate the area of the image that is the ROI
            mask_area_percentage = round(
                100 * sum(sum(image_data.image_mask)) / (image_data.ds_image_size_x * image_data.ds_image_size_y), 3)

            # diagnostic_data.append(mask_area_percentage)

            # Calculate the percentage of the ROI that is bright
            bright_area_percentage = round(100 * sum(sum(bright_area_mask)) / sum(sum(image_data.image_mask)), 3)

            # diagnostic_data.append(bright_area_percentage)

            # If the bright area is too large, exclude it from the mask
            if bright_area_percentage > self.bap_cutoff_percentage and mask_area_percentage < self.map_cutoff_percentage:

                # Modify the mask to remove small areas and increase effectiveness
                bright_area_mask = morphologyEx(bright_area_mask, MORPH_ERODE, kernel=ones((2, 2)), iterations=1)
                bright_area_mask = morphologyEx(bright_area_mask, MORPH_DILATE, kernel=ones((3, 3)), anchor=(1, 1),
                                                iterations=3)

                # Combine the two masks
                mask_for_object = ((image_data.image_mask - bright_area_mask) == 1).astype(uint8)
                # diagnostic_data.append('BA Excluded')
            else:
                mask_for_object = image_data.image_mask
                # diagnostic_data.append('BA Included')

            # Create a blank mask for the first two images
            if len(self.previous_image_masks) == 0:
                gradient_composite_previous_image_mask = ones(mask_for_object.shape).astype(uint8)
                composite_previous_image_mask = gradient_composite_previous_image_mask
            else:
                # Build the single previous image mask
                composite_previous_image_mask = zeros(mask_for_object.shape)
                for single_mask in self.previous_image_masks:
                    composite_previous_image_mask = composite_previous_image_mask + single_mask
                gradient_composite_previous_image_mask = composite_previous_image_mask / len(self.previous_image_masks)
                composite_previous_image_mask = (
                        gradient_composite_previous_image_mask > self.composite_mask_cutoff_value).astype(uint8)

            # diagnostic_data.append(('Gradient Mask', gradient_composite_previous_image_mask))

            if self.segmentation_phase == REST_PHASE:
                # Combine the masks
                mask_for_object = ((mask_for_object + composite_previous_image_mask) == 2).astype(uint8)
            elif self.segmentation_phase == GROWTH_PHASE:
                pass
            else:
                raise Exception(self.segmentation_phase + ' is not a recognized phase.')

            # Save the resulting mask to use
            image_data.post_processed_mask = mask_for_object

            # Update the previous image masks for the next iteration
            self.previous_image_masks.append(image_data.image_mask)
            if len(self.previous_image_masks) > self.num_previous_masks_to_combine:
                self.previous_image_masks.pop(0)

            # return diagnostic_data

        else:
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
        if self.sure_foreground_creation_dice_score is None and \
                self.sure_foreground_creation_kernel_size is None and \
                self.sure_foreground_creation_iterations is not None:
            image_data.sure_foreground_mask = cv2.morphologyEx(
                image_data.post_processed_mask, cv2.MORPH_ERODE,
                np.ones((3, 3), np.uint8), iterations=self.sure_foreground_creation_iterations,  # 4 # 10, # 2
                anchor=(1, 1)
            )
        elif self.sure_foreground_creation_dice_score is not None and \
                self.sure_foreground_creation_kernel_size is not None and \
                self.sure_foreground_creation_iterations is None:
            image_data.sure_foreground_mask = create_dice_score_mask(image_data.post_processed_mask,
                                                                     self.sure_foreground_creation_dice_score,
                                                                     given_kernel_size=self.sure_foreground_creation_kernel_size,
                                                                     mandate_minimum_one_iteration=True)

        else:
            raise Exception('Issue creating foreground mask')

    def create_sure_background_mask(self, image_data: ImageData):
        """
        Create a mask representing the area of the image that is surely the background by
        expanding the segmentation result and inverting the mask.

        Parameters
        ----------
        image_data
            the ImageData object containing the image mask.
        """
        if self.sure_background_creation_dice_score is None and \
                self.sure_background_creation_kernel_size is None and \
                self.sure_background_creation_iterations is not None:
            image_data.sure_background_mask = 1 - cv2.morphologyEx(
                image_data.post_processed_mask, cv2.MORPH_DILATE,
                np.ones((3, 3), np.uint8), iterations=self.sure_background_creation_iterations,  # 6 # 10
                anchor=(1, 1)
            )
        elif self.sure_background_creation_dice_score is not None and \
                self.sure_background_creation_kernel_size is not None and \
                self.sure_background_creation_iterations is None:
            image_data.sure_background_mask = 1 - create_dice_score_mask(image_data.post_processed_mask,
                                                                         self.sure_background_creation_dice_score,
                                                                         given_kernel_size=self.sure_background_creation_kernel_size,
                                                                         mandate_minimum_one_iteration=True)
        else:
            raise Exception('Issue creating background mask')

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

        # Do not create a new previous-image-mask if the flag is set as True
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

    def update_segmentation_phase(self, new_phase: str):
        if new_phase in [GROWTH_PHASE, REST_PHASE]:
            self.segmentation_phase = new_phase
            return True, 'No Error'
        else:
            return False, 'Unrecognized phase'

    def has_segmentation_stabilized(self) -> bool:
        """Determine if the segmentation has stabilized by determining if the number of pixels within each stored
        previous image mask is within the allowed variation from the mean number of pixels."""

        # If a full set of previous image masks has not been stored, return false
        if len(self.previous_image_masks) != self.num_previous_masks_to_combine:
            return False

        # Find the number of pixels in each previous image mask
        previous_mask_sums = [sum(sum(mask)) for mask in self.previous_image_masks]

        # Calculate the average number of pixels in each mask
        avg_sum = sum(previous_mask_sums) / len(previous_mask_sums)

        # If any previous mask sum is outside the allowed variation, return False
        for single_sum in previous_mask_sums:
            if single_sum < avg_sum * (1 - self.allowed_variation_in_stability) or \
                    single_sum > avg_sum * (1 + self.allowed_variation_in_stability):
                return False

        return True
