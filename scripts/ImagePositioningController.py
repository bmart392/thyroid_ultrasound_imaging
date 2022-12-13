from ImageData import ImageData

class ImagePositioningController:
    def __init__(self, x_resolution=0.00025, z_resolution=0.00027):
        self.x_resolution = x_resolution  # m/pixel
        self.z_resolution = z_resolution  # m/pixel

    @staticmethod
    def calculate_error_in_pixels(centroids, image_size_x):
        half_image_width = round(image_size_x / 2)

        position_errors = []
        for centroid in centroids:
            current_error = half_image_width - centroid[0]
            position_errors.append(current_error)

        return position_errors

    def calculate_error_in_meters(self, image_data: ImageData):
        pixel_errors = self.calculate_error_in_pixels(image_data.contour_centroids, image_data.image_size_x)

        distance_errors = []
        for error in pixel_errors:
            distance_errors.append(error * self.x_resolution)

        return distance_errors

    @staticmethod
    def calculate_control_input(positioning_error):
        return []
