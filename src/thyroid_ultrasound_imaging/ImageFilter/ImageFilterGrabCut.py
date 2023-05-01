"""
Define object class for GrabCut based image filters.
"""

# Import the super-class for all image filters
from thyroid_ultrasound_imaging.ImageFilter.ImageFilter import *

from thyroid_ultrasound_imaging.Boundaries.create_convex_triangles_from_points import \
    create_convex_triangles_from_points
from thyroid_ultrasound_imaging.Boundaries.create_mask_array_from_triangles import create_mask_array_from_triangles
from thyroid_ultrasound_imaging.UserInput.user_input_polygon_points import user_input_polygon_points


class ImageFilterGrabCut(ImageFilter):
    """
    A class for filter objects that use the GrabCut segmentation technique.
    """

    def __init__(self, previous_mask_array: np.array = None, image_crop_coordinates: iter = None,
                 image_crop_included: bool = False, include_pre_blurring: bool = False,
                 increase_contrast=False,
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

        # Define characteristics of all GrabCut filters
        self.filter_color = COLOR_BGR
        self.previous_image_mask_array = previous_mask_array

    def pre_process_image(self, image_data: ImageData):
        """
        Increase contrast and blur, if included, the given ImageData object. Overrides the super-class definition.

        Parameters
        ----------
        image_data
            the ImageData object containing the image to be pre-processed.
        """
        # Create a temporary copy of the image
        temp_image = copy(image_data.colorized_image)

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
                    iterCount=15,
                    mode=cv2.GC_INIT_WITH_MASK
                    )

        # Modify the image mask to a binary result
        image_data.image_mask = np.where(
            (image_data.segmentation_initialization_mask == cv2.GC_PR_BGD) |
            (image_data.segmentation_initialization_mask == cv2.GC_BGD), 0, 1).astype('uint8')

    def image_mask_post_process(self, image_data: ImageData):
        """
        Do nothing to post-process the image. Overrides the super-class definition.

        Parameters
        ----------
        image_data
            the ImageData object containing the image.
        """
        pass

    @staticmethod
    def create_sure_foreground_mask(image_data: ImageData):
        """
        Create a mask representing the area of the image that is surely the foreground by
        shrinking the segmentation result.

        Parameters
        ----------
        image_data
            the ImageData object containing the image mask.
        """
        image_data.sure_foreground_mask = cv2.morphologyEx(
            image_data.expanded_image_mask, cv2.MORPH_ERODE,
            np.ones((6, 6), np.uint8), iterations=4  # 10, 2
        )

    @staticmethod
    def create_sure_background_mask(image_data: ImageData):
        """
        Create a mask representing the area of the image that is surely the background by
        expanding the segmentation result and inverting the mask.

        Parameters
        ----------
        image_data
            the ImageData object containing the image mask.
        """
        image_data.sure_background_mask = 1 - cv2.morphologyEx(
            image_data.expanded_image_mask, cv2.MORPH_DILATE,
            np.ones((6, 6), np.uint8), iterations=6
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

    def generate_previous_mask_from_user_input(self, image_data: ImageData):
        """
        Allow the user to generate the previous image mask from the given data.

        Parameters
        ----------
        image_data
            The image data containing the image to be used to create the mask.
        """

        # Save the cropped image from the image data to a variable for use within the function
        test_image = image_data.cropped_image

        # Capture the background of the image from the user
        list_of_points_for_background_polygon = user_input_polygon_points(test_image, "background",
                                                                          display_result=True)

        # Capture the foreground of the image from the user
        list_of_points_for_foreground_polygon = user_input_polygon_points(test_image, "foreground",
                                                                          display_result=True)

        # Convert the points of the background and foreground polygons to triangles
        list_of_background_triangles = create_convex_triangles_from_points(list_of_points_for_background_polygon)
        list_of_foreground_triangles = create_convex_triangles_from_points(list_of_points_for_foreground_polygon)

        # Use the polygons to generate the initial mask
        self.previous_image_mask_array = create_mask_array_from_triangles(list_of_background_triangles,
                                                                          list_of_foreground_triangles,
                                                                          test_image.cropped_image.shape[:2])