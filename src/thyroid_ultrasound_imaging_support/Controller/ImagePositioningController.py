from numpy import arctan2, rad2deg, array

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

        # Define the acceptable position error of the centroid in the image for each dimension in the probe's space
        self.acceptable_errors = [0.005,  # meters in x
                                  0.005,  # meters in y
                                  1.01,  # meters in z
                                  10,  # deg in x
                                  10,  # deg in y
                                  1]  # deg in z

    def calculate_resolution(self, image_height: int):

        return (self.imaging_depth / 100) / image_height

    # TODO check how this is calculated and fix it because somehow it gave 50 degrees when the centroid was centered
    def calculate_position_error(self, image_data: ImageData):

        # Declare the error values that cannot be determined from the image
        z_error = 0
        x_angle_error = 0
        y_angle_error = 0

        # Calculate position errors using centroid location in pixels and image size
        if len(image_data.contour_centroids) == 2:

            # In X, balance the centroids across the middle of the image
            centroid_one = image_data.contour_centroids[0]
            centroid_two = image_data.contour_centroids[1]
            x_error = centroid_one[0] - centroid_two[0]

            # In Y, average the centroids distance to the top of the image
            y_error = (centroid_one[1] + centroid_two[1]) / 2

            # In Z angle, balance the centroids across the middle of the image
            centroid_one_angle = arctan2(centroid_one[0], centroid_one[1])
            centroid_two_angle = arctan2(centroid_two[0], centroid_two[1])
            z_angle_error = rad2deg(centroid_one_angle - centroid_two_angle)

        elif len(image_data.contour_centroids) == 1:

            # Rename the centroid for ease of use
            centroid = image_data.contour_centroids[0]

            # In X, measure the distance to the middle of the image
            x_error = (centroid[0] - (image_data.image_size_x / 2)) * self.calculate_resolution(image_data.image_size_y)

            # In Y, measure the distance to the top of the image
            y_error = centroid[1] * self.calculate_resolution(image_data.image_size_y)

            # In Z angle, bring the angle to zero
            z_angle_error = rad2deg(arctan2(centroid[0], centroid[1]))

        else:

            raise Exception("More centroids than expected were found.")

        exact_position_errors = [round(x_error, 5), round(y_error, 5), round(z_error, 5),
                                 round(x_angle_error, 1), round(y_angle_error, 1), round(z_angle_error, 1)]

        # Define a temporary flag to determine if the image is centered
        temp_is_image_centered = True

        # Define a temporary place to store the control inputs
        adjusted_position_errors = []

        # Check if the error is within an acceptable amount
        for single_dimension_error, single_dimension_acceptable_error in zip(exact_position_errors,
                                                                             self.acceptable_errors):

            # Determine if the error is acceptable in a single dimension
            is_dimension_centered = single_dimension_acceptable_error >= abs(single_dimension_error)

            # Calculate the control input for the given dimension
            if not is_dimension_centered:
                adjusted_position_errors.append(single_dimension_error)
            else:
                adjusted_position_errors.append(0)

            # Calculate if the image is centered in all dimensions
            temp_is_image_centered = temp_is_image_centered * is_dimension_centered

        return adjusted_position_errors, temp_is_image_centered

    """def calculate_control_input(self, image_data: ImageData):

        # Calculate position errors using centroid location in pixels and image size
        position_errors = [(image_data.contour_centroids[0][0] - (image_data.image_size_x / 2)) * self.x_resolution,
                           # 0
                           0,
                           # (image_data.contour_centroids[0][0] - (image_data.image_size_x / 2)) * self.x_resolution,
                           0,
                           # -(image_data.contour_centroids[0][1] - (image_data.image_size_y / 2)) * self.z_resolution,
                           0, 0, 0]

        if self.debug_mode:
            print("X error - meters = ", position_errors[0])
            print("X error - pixels = ", position_errors[0] / self.x_resolution)
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

        return control_inputs_msg, position_errors, is_error_acceptable"""
