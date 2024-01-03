#!/usr/bin/env python3

"""
File containing code to ensure even patient contact.
"""

# Import standard ROS packages

# Import standard python packages
from numpy import zeros, linspace, array, polyfit

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_messages.msg import image_data_message, Float64Stamped

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageBalancing.ImageSector import *

# Define constants for use in the node
CURRENT_VALUE: int = int(0)
NEW_VALUE: int = int(1)

LEFT_SECTOR: int = int(0)
RIGHT_SECTOR: int = int(1)

X_POINTS: int = int(0)
Y_POINTS: int = int(1)


class ImageContactBalanceNode(BasicNode):

    def __init__(self, image_field_of_view: float = 30, num_slices: int = 20, image_shape: tuple = (640, 480),
                 imaging_depth_meters: float = 0.05, bottom_distance_to_imaginary_center_meters: float = 0.275,
                 sides_distance_to_imaginary_center_meters: float = 0.074, start_of_us_image: int = 0,
                 end_of_us_image_from_bottom: int = 0, image_center_offset_px: int = 0,
                 down_sampling_rate: int = 5, sector_angles: list = None,
                 skin_level_offset: int = 100,
                 ):

        # Call to super class init
        super().__init__()

        # Check to ensure that the number of slices given is even
        if num_slices % 2 == 1:
            raise Exception("Non-even number of slices given.")

        # Define the start and end angles of every sector in the image
        if sector_angles is None:
            self.sector_angles = linspace(-image_field_of_view / 2, image_field_of_view / 2, num_slices + 1)
        else:
            self.sector_angles = sector_angles

        # Define the down sampling rate of the algorithm
        self.down_sampling_rate = down_sampling_rate

        # Define an offset for finding the skin level
        self.skin_level_offset = skin_level_offset

        # Define a variable to store each sector of the image in
        self.sectors = []

        # Define characteristics of the ultrasound image
        self.imaging_depth_meters = [imaging_depth_meters, imaging_depth_meters]
        self.img_num_rows = [image_shape[0], image_shape[0]]
        self.img_num_cols = [image_shape[1], image_shape[1]]
        self.bottom_distance_to_imaginary_center_meters = [bottom_distance_to_imaginary_center_meters,
                                                           bottom_distance_to_imaginary_center_meters]
        self.sides_distance_to_imaginary_center_meters = [sides_distance_to_imaginary_center_meters,
                                                          sides_distance_to_imaginary_center_meters]
        self.start_of_us_image = [start_of_us_image, start_of_us_image]
        self.end_of_us_image_from_bottom = [end_of_us_image_from_bottom, end_of_us_image_from_bottom]
        self.image_center_offset_px = [image_center_offset_px, image_center_offset_px]

        # Generate the image sectors
        self.rebuild_sectors()

        # Define ranges representing the indices of the sectors on the left and right half of the image
        self.left_half_sectors = range(int(len(self.sectors) / 2))
        self.right_half_sectors = range(int(len(self.sectors) / 2), len(self.sectors))

        # Define the minimum intensity value for the sector to be considered bright
        self.min_intensity = 20

        # Define a variable to store received ultrasound image
        self.new_ultrasound_images = []

        # Define a bitmask to note which pixels were sampled
        self.sampled_data_bit_mask = None

        # Define variables to store the lines of best fit
        self.best_fit_left_line_a = None
        self.best_fit_left_line_b = None
        self.best_fit_right_line_a = None
        self.best_fit_right_line_b = None
        self.best_fit_shared_line_a = None
        self.best_fit_shared_line_b = None

        # Define a list structure to store the points found along the skin
        self.skin_points = [[[], []], [[], []]]

        # Initialize the ROS Node
        init_node("ImageContactBalanceNode")

        # Define a publisher to publish the error
        self.patient_contact_error_publisher = Publisher(RC_PATIENT_CONTACT_ERROR, Float64Stamped, queue_size=1)

        # Define a publisher to publish if the patient is in contact
        self.patient_contact_status_publisher = Publisher(IMAGE_PATIENT_CONTACT, Bool, queue_size=1)

        # Define a subscriber to listen for the raw image
        Subscriber(IMAGE_RAW, image_data_message, self.new_raw_image_callback)

        # TODO define a way to publish information so that the results can be visualized

    ###############
    # ROS Callbacks
    # region
    def new_raw_image_callback(self, data: image_data_message):
        """
        Receives a new ultrasound image and saves it. If too many ultrasound images have been saved,
        the oldest image is purged.

        Parameters
        ----------
        data
            The message containing the original ultrasound image.
        """

        # Create a temporary ImageData object from the message
        temp_image_data_object = ImageData(image_data_msg=data)

        # Save the original ultrasound image
        self.new_ultrasound_images.append(temp_image_data_object.original_image)

        # If the list of new ultrasound images is too long, remove the oldest image
        if len(self.new_ultrasound_images) > 5:
            self.new_ultrasound_images.pop(0)

    # endregion
    ###############

    ##################
    # Helper Functions
    # region
    def rebuild_sectors(self):
        """
        Builds the image sectors using the current values unless new values are available.
        """
        # Clear existing sector data
        self.sectors = []

        # Determine which parameters have changed and use the newest values
        imaging_depth_meters_to_use = self.imaging_depth_meters[CURRENT_VALUE]
        if not imaging_depth_meters_to_use == self.imaging_depth_meters[NEW_VALUE]:
            imaging_depth_meters_to_use = self.imaging_depth_meters[NEW_VALUE]
            self.imaging_depth_meters[CURRENT_VALUE] = self.imaging_depth_meters[NEW_VALUE]

        img_num_rows_to_use = self.img_num_rows[CURRENT_VALUE]
        if not img_num_rows_to_use == self.img_num_rows[NEW_VALUE]:
            img_num_rows_to_use = self.img_num_rows[NEW_VALUE]
            self.img_num_rows[CURRENT_VALUE] = self.img_num_rows[NEW_VALUE]

        img_num_cols_to_use = self.img_num_cols[CURRENT_VALUE]
        if not img_num_cols_to_use == self.img_num_cols[NEW_VALUE]:
            img_num_cols_to_use = self.img_num_cols[NEW_VALUE]
            self.img_num_cols[CURRENT_VALUE] = self.img_num_cols[NEW_VALUE]

        bottom_distance_to_imaginary_center_meters_to_use = self.bottom_distance_to_imaginary_center_meters[
            CURRENT_VALUE]
        if not bottom_distance_to_imaginary_center_meters_to_use == \
               self.bottom_distance_to_imaginary_center_meters[NEW_VALUE]:
            bottom_distance_to_imaginary_center_meters_to_use = \
                self.bottom_distance_to_imaginary_center_meters[NEW_VALUE]
            self.bottom_distance_to_imaginary_center_meters[CURRENT_VALUE] = \
                self.bottom_distance_to_imaginary_center_meters[NEW_VALUE]

        sides_distance_to_imaginary_center_meters_to_use = self.sides_distance_to_imaginary_center_meters[CURRENT_VALUE]
        if not self.sides_distance_to_imaginary_center_meters[CURRENT_VALUE] == \
               self.sides_distance_to_imaginary_center_meters[NEW_VALUE]:
            sides_distance_to_imaginary_center_meters_to_use = self.sides_distance_to_imaginary_center_meters[NEW_VALUE]
            self.sides_distance_to_imaginary_center_meters[CURRENT_VALUE] = \
                self.sides_distance_to_imaginary_center_meters[NEW_VALUE]

        start_of_us_image_to_use = self.start_of_us_image[CURRENT_VALUE]
        if not self.start_of_us_image[CURRENT_VALUE] == self.start_of_us_image[NEW_VALUE]:
            start_of_us_image_to_use = self.start_of_us_image[NEW_VALUE]
            self.start_of_us_image[CURRENT_VALUE] = self.start_of_us_image[NEW_VALUE]

        end_of_us_image_from_bottom_to_use = self.end_of_us_image_from_bottom[CURRENT_VALUE]
        if not self.end_of_us_image_from_bottom[CURRENT_VALUE] == self.end_of_us_image_from_bottom[NEW_VALUE]:
            end_of_us_image_from_bottom_to_use = self.end_of_us_image_from_bottom[NEW_VALUE]
            self.end_of_us_image_from_bottom[CURRENT_VALUE] = self.end_of_us_image_from_bottom[NEW_VALUE]

        image_center_offset_px_to_use = self.image_center_offset_px[CURRENT_VALUE]
        if not image_center_offset_px_to_use == self.image_center_offset_px[NEW_VALUE]:
            image_center_offset_px_to_use = self.image_center_offset_px[NEW_VALUE]
            self.image_center_offset_px[CURRENT_VALUE] = self.image_center_offset_px[NEW_VALUE]

        # Generate the image sectors
        for ii in range(len(self.sector_angles) - 1):
            self.sectors.append(
                ImageSector(
                    (self.sector_angles[ii], self.sector_angles[ii + 1]),
                    imaging_depth_meters=imaging_depth_meters_to_use,
                    image_num_rows=img_num_rows_to_use,
                    image_num_cols=img_num_cols_to_use,
                    bottom_distance_to_imaginary_center_meters=bottom_distance_to_imaginary_center_meters_to_use,
                    sides_distance_to_imaginary_center_meters=sides_distance_to_imaginary_center_meters_to_use,
                    top_of_ultrasound_image_offset_px=start_of_us_image_to_use,
                    bottom_of_ultrasound_image_offset_px=end_of_us_image_from_bottom_to_use,
                    image_center_offset_px=image_center_offset_px_to_use,
                )
            )

    # endregion
    ##################
    def main_loop(self):
        """
        Whenever there is a new ultrasound image, the image sectors are updated, the image is analyzed,
        and then the image is published.
        """

        # Reset variables
        self.sampled_data_bit_mask = None
        self.skin_points = [[[], []], [[], []]]

        # If there is a new ultrasound image
        if len(self.new_ultrasound_images) > 0:

            # Pop out the latest image
            latest_image = self.new_ultrasound_images.pop(-1)

            # Save the image shape in case it has changed
            self.img_num_rows[NEW_VALUE] = latest_image.shape[0]
            self.img_num_cols[NEW_VALUE] = latest_image.shape[1]

            # Rebuild the sectors if the imaging depth has changed
            self.rebuild_sectors()

            # Define the bitmask to use to show the sampled points
            self.sampled_data_bit_mask = zeros((self.img_num_rows[CURRENT_VALUE],
                                                self.img_num_cols[CURRENT_VALUE]),
                                               'uint8')

            # Define a variable to use to store the last sector where a point was found
            last_sector_used = 0

            # Iterate through the rows of the original image that contain the ultrasound image
            for yy in range(self.start_of_us_image[CURRENT_VALUE],
                            self.img_num_rows[CURRENT_VALUE] - self.end_of_us_image_from_bottom[CURRENT_VALUE],
                            self.down_sampling_rate):

                # Iterate through the columns that are between the left line of the leftmost column and the right
                # line of the rightmost column skipping points based on the down-sampling rate
                for xx in range(round(self.sectors[0].left_line.calc_point_on_line(y_value=yy)) - 2,
                                round(self.sectors[-1].right_line.calc_point_on_line(y_value=yy)) + 2,
                                self.down_sampling_rate):

                    # Window the acceptable values of x between 0 and the number of columns in the image
                    if xx < 0:
                        xx = 0
                    if xx > self.img_num_cols[CURRENT_VALUE] - 1:
                        xx = self.img_num_cols[CURRENT_VALUE] - 1

                    # For each sector, starting at the sector in which the last point was found
                    for jj in range(last_sector_used, len(self.sectors)):

                        # Check if the point is in the sector
                        if self.sectors[jj].is_point_in_sector((xx, yy), latest_image[yy][xx]):
                            # Update the bitmap correspondingly
                            # self.sampled_data_bit_mask[yy, xx] = uint8(1)

                            # Update the value of the last sector used
                            last_sector_used = jj

                            break

                # Reset the value of the last sector used
                last_sector_used = 0

            # Define variables to store the number of dark sectors in each half of the image
            num_dark_sectors = [0, 0]

            # For each sector in the left half area,
            for li, ri in zip(self.left_half_sectors, self.right_half_sectors):

                # If the average intensity is lower than the minimum
                if self.sectors[li].avg_data_value < self.min_intensity:
                    # Increment the counter
                    num_dark_sectors[0] = num_dark_sectors[0] + 1
                else:
                    # Note that the sector is bright
                    self.sectors[li].is_bright = True

                # If the average intensity is lower than the minimum
                if self.sectors[ri].avg_data_value < self.min_intensity:
                    # Increment the counter
                    num_dark_sectors[1] = num_dark_sectors[1] + 1
                else:
                    # Note that the sector is bright
                    self.sectors[ri].is_bright = True

            # Calculate the difference in the number of dark sectors on each side of the image
            dark_sector_difference = (num_dark_sectors[1] - num_dark_sectors[0]) / 10  # Scale down the error

            # If patient contact is detected at all
            if abs(dark_sector_difference) < floor(len(self.sectors) * 0.5):

                # Publish the true patient contact message
                patient_contact_status = True

                # Calculate the image error based on the number of dark sectors
                patient_contact_error = float(dark_sector_difference)

                # Check if there is enough patient contact to calculate the error from the skin layer
                if abs(dark_sector_difference) < floor((len(self.sectors) * 0.1)):

                    # Define an iterator for the half-sector
                    ii = 0

                    # For each half-sector
                    for half_sector in [self.left_half_sectors, self.right_half_sectors]:

                        # For each sector in each half-sector
                        for u in half_sector:

                            # Make sure the sector is bright
                            if self.sectors[u].is_bright and not u == 0:

                                # Define a variable to store the number of points that qualify along each line
                                num_points_found_over_min = 0

                                # For each y value in the top section of the ultrasound image
                                for yyy in range(self.start_of_us_image[CURRENT_VALUE] + self.skin_level_offset,
                                                 self.img_num_rows[CURRENT_VALUE] -
                                                 self.end_of_us_image_from_bottom[
                                                     CURRENT_VALUE],
                                                 1):

                                    # Calculate the corresponding x value
                                    xxx = round(self.sectors[u].left_line.calc_point_on_line(y_value=yyy))

                                    # If the intensity of that pixel is bright enough
                                    if latest_image[yyy][xxx] > 2 * self.min_intensity:

                                        # Increment the number of points found
                                        num_points_found_over_min = num_points_found_over_min + 1

                                        # If enough points have been found
                                        if num_points_found_over_min > 5:
                                            # Save the (x, y) coordinate of the point
                                            self.skin_points[ii][X_POINTS].append(xxx)
                                            self.skin_points[ii][Y_POINTS].append(yyy)

                                            # Break and move on to the next sector
                                            break
                                    # If the intensity is not bright enough, restart the count
                                    else:
                                        num_points_found_over_min = 0

                        # Increment the half-sector count
                        ii = ii + 1

                    """# Calculate the straight line of best fit for the left half-sector
                    # noinspection PyTupleAssignmentBalance
                    self.best_fit_left_line_a, self.best_fit_left_line_b = polyfit(
                        array(self.skin_points[LEFT_SECTOR][X_POINTS]),
                        array(self.skin_points[LEFT_SECTOR][Y_POINTS]),
                        1)

                    # Calculate the straight line of best fit for the right half-sector
                    # noinspection PyTupleAssignmentBalance
                    self.best_fit_right_line_a, self.best_fit_right_line_b = polyfit(
                        array(self.skin_points[RIGHT_SECTOR][X_POINTS]),
                        array(self.skin_points[RIGHT_SECTOR][Y_POINTS]), 1)"""

                    if len(self.skin_points[LEFT_SECTOR][X_POINTS]) > 0 and len(self.skin_points[RIGHT_SECTOR][X_POINTS]) > 0:

                        # Calculate the straight line of best fit for the entire image
                        # noinspection PyTupleAssignmentBalance
                        self.best_fit_shared_line_a, self.best_fit_shared_line_b = polyfit(
                            array(self.skin_points[LEFT_SECTOR][X_POINTS] +
                                  self.skin_points[RIGHT_SECTOR][X_POINTS]),
                            array(self.skin_points[LEFT_SECTOR][Y_POINTS] +
                                  self.skin_points[RIGHT_SECTOR][Y_POINTS]), 1)

                        # Add the skin layer error to the image error
                        patient_contact_error = round(self.best_fit_shared_line_a, 3) * 5  # Scale the error up

            else:
                # If zero patient contact is detected, publish the contact message accordingly
                patient_contact_status = False

                # Publish zero error
                patient_contact_error = 0.0

            # Publish the patient contact and image error
            self.patient_contact_status_publisher.publish(Bool(patient_contact_status))
            patient_contact_error_msg = Float64Stamped()
            patient_contact_error_msg.header.stamp = Time.now()
            patient_contact_error_msg.data.data = patient_contact_error
            self.patient_contact_error_publisher.publish(patient_contact_error_msg)


if __name__ == '__main__':
    # Create the node
    node = ImageContactBalanceNode()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # While the node is not shut down
    while not is_shutdown():

        # Run the main code of the node
        node.main_loop()

    print("Node terminated.")
