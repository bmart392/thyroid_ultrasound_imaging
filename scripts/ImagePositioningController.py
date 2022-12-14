from ImageData import ImageData
from numpy import radians
from geometry_msgs.msg import TwistStamped


class ImagePositioningController:
    def __init__(self, x_resolution=0.00025, z_resolution=0.00027):
        self.x_resolution = x_resolution  # m/pixel
        self.z_resolution = z_resolution  # m/pixel
        self.acceptable_error = [0.05, 0.05, 0.05, radians(10), radians(10), radians(10)]  # [m, m, m, rads, rads, rads]

        self.control_input_gains = [
            # [k_p, k_d]
            [1, 0],  # x
            [1, 0],  # y
            [1, 0],  # z
            [1, 0],  # roll
            [1, 0],  # pitch
            [1, 0],  # yaw
        ]

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

    def calculate_control_input(self, image_data: ImageData):

        # Calculate position errors using centroid location in pixels and image size
        position_errors = [0,
                           (image_data.contour_centroids[0][0] - image_data.image_size_x) * self.x_resolution,
                           -(image_data.contour_centroids[0][1] - image_data.image_size_y) * self.z_resolution,
                           0, 0, 0]

        # Define array to store the new control inputs
        control_inputs = []

        # Define variable to store if the image is centered
        is_error_acceptable = True

        # Calculate control inputs from centroid positions and check if error is too large
        for current_error, gains, error_limit in zip(position_errors, self.control_input_gains, self.acceptable_error):
            control_inputs.append(-current_error * gains[0])
            is_error_acceptable = is_error_acceptable + (error_limit >= abs(current_error))

        # Generate twist message to send control inputs
        control_inputs_msg = TwistStamped()
        control_inputs_msg.twist.linear.x = control_inputs[0]
        control_inputs_msg.twist.linear.y = control_inputs[1]
        control_inputs_msg.twist.linear.z = control_inputs[2]
        control_inputs_msg.twist.angular.x = control_inputs[3]
        control_inputs_msg.twist.angular.y = control_inputs[4]
        control_inputs_msg.twist.angular.z = control_inputs[5]

        return control_inputs_msg, position_errors, is_error_acceptable
