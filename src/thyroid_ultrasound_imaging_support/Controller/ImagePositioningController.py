from numpy import arctan2, rad2deg, array, ceil

from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData


class ImagePositioningController:
    def __init__(self, imaging_depth=6, debug_mode=False, analysis_mode=False):
        """
        Create a class for generating a control input using a filtered ultrasound image.

        Parameters
        ----------
        imaging_depth: float
            the imaging depth of the filtered image in centimeters.

        debug_mode: bool
            display graphics and additional print statements helpful in the debugging process.

        analysis_mode: bool
            display the time key processes take to occur.

        """
        self.debug_mode = debug_mode
        self.analysis_mode = analysis_mode

        self.imaging_depth = imaging_depth  # cm

        self.set_point_offset = 0

        # Define the acceptable position error of the centroid in the image for each dimension in the probe's space
        self.acceptable_errors = [20,  # pixels in x
                                  0.005,  # pixels in y
                                  1.01,  # pixels in z
                                  10,  # deg in x
                                  10,  # deg in y
                                  1]  # deg in z

    def calculate_resolution(self, image_height: int):

        return (self.imaging_depth / 100) / image_height

    def calculate_position_error(self, image_data: ImageData):

        # Declare the error values that cannot be determined from the image
        y_error = 0
        z_error = 0
        x_angle_error = 0
        y_angle_error = 0
        z_angle_error = 0

        # Calculate position errors using centroid location in pixels and image size
        if False and len(image_data.contour_centroids) > 1:

            # In X in the image frame, balance the centroids across the middle of the image
            centroid_one = image_data.contour_centroids[0]
            centroid_two = image_data.contour_centroids[1]
            x_error = centroid_one[0] - centroid_two[0]

        elif len(image_data.contour_centroids) == 1:

            # Rename the centroid for ease of use
            centroid = image_data.contour_centroids[0]

            # In X in the image frame, measure the distance to the middle of the image
            center_offset = ceil(self.set_point_offset * image_data.ds_image_size_x)
            center = (image_data.ds_image_size_x / 2) + center_offset
            distance_from_center = centroid[0] - center

            x_error = centroid[0] - image_data.ds_image_size_x * (self.set_point_offset + 0.5)

        else:

            raise Exception(str(len(image_data.contour_centroids)) + " centroids were found included in the image. ")

        # Send the position errors relative to the camera frame
        exact_position_errors = [round(x_error, 5), round(y_error, 5), round(z_error, 5),
                                 round(x_angle_error, 1), round(y_angle_error, 1), round(z_angle_error, 1)]

        return exact_position_errors

