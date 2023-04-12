from numpy import radians

from geometry_msgs.msg import TwistStamped
from scripts.b_ImageData.ImageData import ImageData


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

        imaging_depth = imaging_depth  # cm

        # Calculate the resolution of the image
        resolution = (imaging_depth / 100) / 480

        # Set the image resolution in each direction
        self.x_resolution = resolution  # m/pixel
        self.z_resolution = resolution  # m/pixel

        # Define the acceptable position error of the centroid in the image for each dimension in the probe's space
        self.acceptable_error = [0.005,  # meters in x
                                 1.01,  # meters in y
                                 1.01,  # meters in z
                                 radians(10),  # rads
                                 radians(10),  # rads
                                 radians(10)]  # rads

        # Define the control input gains for each dimension
        self.control_input_gains = [
            # [k_p, k_d]
            [0.5, 0.0],  # x
            [1.0, 0.0],  # y
            [1.0, 0.0],  # z
            [1.0, 0.0],  # roll
            [1.0, 0.0],  # pitch
            [1.0, 0.0],  # yaw
        ]

    def calculate_control_input(self, image_data: ImageData):

        # Calculate position errors using centroid location in pixels and image size
        position_errors = [(image_data.contour_centroids[0][0] - (image_data.image_size_x / 2)) * self.x_resolution,  # 0
                           0,  # (image_data.contour_centroids[0][0] - (image_data.image_size_x / 2)) * self.x_resolution,
                           0,  # -(image_data.contour_centroids[0][1] - (image_data.image_size_y / 2)) * self.z_resolution,
                           0, 0, 0]

        if self.debug_mode:
            print("X error - meters = ", position_errors[0])
            print("X error - pixels = ", position_errors[0]/self.x_resolution)
            print("X error - limit = ", self.acceptable_error[0])
            print("X error - is acceptable? = ", self.acceptable_error[0] >= abs(position_errors[0]))
            print("X control input = ", position_errors[0] * self.control_input_gains[0][0])

        # Define array to store the new control inputs
        control_inputs = []

        # Define variable to store if the image is centered
        is_error_acceptable = True

        # Calculate control inputs from centroid positions and check if error is too large
        for current_error, gains, error_limit in zip(position_errors, self.control_input_gains, self.acceptable_error):
            control_inputs.append(current_error * gains[0])
            is_error_acceptable = is_error_acceptable * (error_limit >= abs(current_error))

        # Generate twist message to send control inputs
        control_inputs_msg = TwistStamped()
        control_inputs_msg.twist.linear.x = control_inputs[0]
        control_inputs_msg.twist.linear.y = control_inputs[1]
        control_inputs_msg.twist.linear.z = control_inputs[2]
        control_inputs_msg.twist.angular.x = control_inputs[3]
        control_inputs_msg.twist.angular.y = control_inputs[4]
        control_inputs_msg.twist.angular.z = control_inputs[5]

        return control_inputs_msg, position_errors, is_error_acceptable
