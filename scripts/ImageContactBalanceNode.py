#!/usr/bin/env python3

"""
File containing code to ensure even patient contact.
"""

# Import standard ROS packages

# Import standard python packages
from numpy import floor, sin, deg2rad, zeros, newaxis, uint8, linspace
from cv2 import imread, cvtColor, COLOR_GRAY2BGR
from cv_bridge import CvBridge
from matplotlib.pyplot import imshow, figure, show, plot
from matplotlib.pyplot import Circle as circ

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_messages.msg import image_data_message

# Import custom python packages
from thyroid_ultrasound_imaging_support.Visualization.display_process_timer import *
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData


# Define constants for use in the node
CURRENT_VALUE: int = int(0)
NEW_VALUE: int = int(1)


class ImageContactBalanceNode(BasicNode):

    def __init__(self, image_field_of_view: float = 30, num_slices: int = 20, image_shape: tuple = (640, 480)):

        # Call to super class init
        super().__init__()

        # Define the start and end angles of every sector in the image
        self.sector_angles = linspace(-image_field_of_view / 2, image_field_of_view / 2, num_slices + 1)
        # self.sector_angles = [-image_field_of_view / 2, image_field_of_view / 2]

        # Define a variable to store each sector of the image in
        self.sectors = []

        # Define characteristics of the ultrasound image
        self.imaging_depth_meters = [0.05, 0.05]
        self.img_num_rows = [image_shape[0], image_shape[0]]
        self.img_num_cols = [image_shape[1], image_shape[1]]
        self.bottom_distance_to_imaginary_center_meters = [0.275, 0.275]
        self.sides_distance_to_imaginary_center_meters = [0.074, 0.074]
        self.start_of_us_image = [0, 0]
        self.end_of_us_image_from_bottom = [0, 0]
        self.image_center_offset_px = [0, 0]

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

        # Initialize the ROS Node
        init_node("ImageContactBalanceNode")

        # Define a publisher to publish the error
        self.error_publisher = Publisher('/contact_angle_error', Float64, queue_size=1)

        # Define a subscriber to listen for the raw image
        Subscriber(IMAGE_RAW, image_data_message, self.new_raw_image_callback)

        # Define a subscriber to listen for if the patient is in contact

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

            # Define the down-sampling rate for the image
            down_sampling_rate = 5

            # Iterate through the rows of the original image that contain the ultrasound image
            for yy in range(self.start_of_us_image[CURRENT_VALUE],
                            self.img_num_rows[CURRENT_VALUE] - self.end_of_us_image_from_bottom[CURRENT_VALUE],
                            down_sampling_rate):

                # Iterate through the columns that are between the left line of the leftmost column and the right line
                # of the rightmost column skipping points based on the down-sampling rate
                for xx in range(round(self.sectors[0].left_line.calc_point_on_line(y_value=yy)) - 2,
                                round(self.sectors[-1].right_line.calc_point_on_line(y_value=yy)) + 2,
                                down_sampling_rate):

                    if xx < 0:
                        xx = 0
                    if xx > self.img_num_cols[CURRENT_VALUE] - 1:
                        xx = self.img_num_cols[CURRENT_VALUE] - 1

                    # For each sector, starting at the sector in which the last point was found
                    for jj in range(last_sector_used, len(self.sectors)):
                        a = self.sectors[jj]
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
            for li in self.left_half_sectors:

                # If the average intensity is lower than the minimum
                if self.sectors[li].avg_data_value < self.min_intensity:
                    # Increment the counter
                    num_dark_sectors[0] = num_dark_sectors[0] + 1

            # For each sector in the right half area,
            for ri in self.right_half_sectors:

                # If the average intensity is lower than the minimum
                if self.sectors[ri].avg_data_value < self.min_intensity:
                    # Increment the counter
                    num_dark_sectors[1] = num_dark_sectors[1] + 1

            # Publish the difference between the two numbers as the error
            # Float64(float(num_dark_sectors[1] - num_dark_sectors[0]))

            # print("Number of dark sectors: " + str(sum(num_dark_sectors)))
            # print("Error: " + str(num_dark_sectors[1] - num_dark_sectors[0]))
            # print("---")


class ImageSector:

    def __init__(self, line_angles: tuple,
                 imaging_depth_meters: float,
                 image_num_rows: int, image_num_cols: int,
                 bottom_distance_to_imaginary_center_meters: float,
                 sides_distance_to_imaginary_center_meters: float,
                 top_of_ultrasound_image_offset_px: int = 0,
                 bottom_of_ultrasound_image_offset_px: int = 0,
                 image_center_offset_px: int = 0):
        """
        Creates a pie-shaped sector of an ultrasound image.

        Parameters
        ----------
        line_angles
            The starting and ending angles for the sector edges. Order is not important.
        imaging_depth_meters
            The imaging depth of the ultrasound image measured in meters.
        image_num_rows
            The number of rows in the ultrasound image array.
        image_num_cols
            The number of columns in the ultrasound image array.
        sides_distance_to_imaginary_center_meters
            The distance from the top of the ultrasound image to the imaginary center.
        top_of_ultrasound_image_offset_px
            The distance from the top of the image array to the start of the actual ultrasound image.
        bottom_of_ultrasound_image_offset_px
            The distance from the bottom of the actual ultrasound image to the bottom of the image array.
        image_center_offset_px
            The offset from the center of the image to the center of the ultrasound image.
        """

        # Ensure that the sector is only defined with two angles
        if len(line_angles) > 2:
            raise Exception("Too many angles passed into function.")

        # Always define the left angle as the larger angle
        left_line_angle = min(line_angles)

        # Always define the right angle as the smaller angle
        right_line_angle = max(line_angles)

        if left_line_angle <= 0:
            left_line_directionality = True
        else:
            left_line_directionality = False

        if right_line_angle <= 0:
            right_line_directionality = False
        else:
            right_line_directionality = True

        # Calculate the resolution of the image
        pixels_per_meter = image_num_rows / imaging_depth_meters  # px/m

        # Calculate the number of pixels to the imaginary center for the side edges
        sides_pixel_to_imaginary_center = round(sides_distance_to_imaginary_center_meters * pixels_per_meter)

        # Calculate the number of pixels to the imaginary center for the bottom edge
        bottom_pixel_to_imaginary_center = round(bottom_distance_to_imaginary_center_meters * pixels_per_meter)

        # Calculate the center of the image
        image_center = floor(image_num_cols / 2) + image_center_offset_px

        # Define the two lines used to define the sides of the sector
        self.left_line = Line(
            (
                round(image_center + (sides_pixel_to_imaginary_center * sin(deg2rad(left_line_angle)))),
                top_of_ultrasound_image_offset_px
            ),
            (
                image_center, top_of_ultrasound_image_offset_px - sides_pixel_to_imaginary_center
            ),
            left_line_directionality)

        self.right_line = Line(
            (
                round(image_center + (sides_pixel_to_imaginary_center * sin(deg2rad(right_line_angle)))),
                top_of_ultrasound_image_offset_px
            ),
            (
                image_center, top_of_ultrasound_image_offset_px - sides_pixel_to_imaginary_center
            ),
            right_line_directionality)

        # Save the top upper limit of the set
        self.top_edge = top_of_ultrasound_image_offset_px

        # Define the circle used to define the lower edge of the set
        self.bottom_edge_circle = Circle(
            (image_center, top_of_ultrasound_image_offset_px - bottom_pixel_to_imaginary_center),
            image_num_rows - top_of_ultrasound_image_offset_px +
            bottom_pixel_to_imaginary_center - bottom_of_ultrasound_image_offset_px)

        # Create variables to store data about the set
        self.avg_data_value = 0.
        self.num_data_values = 0.

    def is_point_in_sector(self, pt: tuple, pt_value: int) -> bool:
        """
        Checks if the given point is included in the sector and updates the sector data if the pt is part of the sector.
        Returns True if the point is in the sector.

        Parameters
        ----------
        pt
            The (x, y) point to check.
        pt_value
            The value stored at the given point.
        """

        # If the point is in both line sets, below the upper boundary, and above the lower boundary
        if self.left_line.is_point_in_set(pt) and self.right_line.is_point_in_set(pt) and \
                pt[1] > self.top_edge and self.bottom_edge_circle.is_point_in_set(pt):
            # Calculate the new average value for the sector
            self.avg_data_value = ((self.avg_data_value * self.num_data_values) + pt_value) / (self.num_data_values + 1)

            # Increment the number of data values that are in the sector
            self.num_data_values = self.num_data_values + 1

            # Return that the point is in the sector.
            return True

        # Returns that the point is not in the sector.
        return False


class Line:

    def __init__(self, pt_1: tuple, pt_2: tuple, directionality: bool = False):
        """
        Creates a 2D set bounded by a single line, defined by two points, with the given directionality.
        The equation of a line used for this object is: 0 = ax + by + c

        False directionality means that a point is in set when solution to the equation of the line is negative.
        In other words, when the point is below the line when shown on standard 2D cartesian plot.

        True directionality means that a point is in set when solution to the equation of the line is positive.
        In other words, when the point is above the line when shown on standard 2D cartesian plot.

        Parameters
        ----------
        pt_1:
            The first point used to define the line.
        pt_2:
            The second point used to define the line.
        directionality:
            Determines which direction that points are considered in the set.
        """

        # Save the points used to generate the line
        self.pt_1 = pt_1
        self.pt_2 = pt_2

        # Check if the line is horizontal
        if pt_2[1] - pt_1[1] == 0:
            self.a_constant = 0
            self.b_constant = 1
            self.c_constant = -pt_2[1]

        # Check if the line is vertical
        elif pt_2[0] - pt_1[0] == 0:
            self.a_constant = 1
            self.b_constant = 0
            self.c_constant = -pt_2[0]

        # Otherwise calculate the slope and y-intercept
        else:
            self.a_constant = -(pt_2[1] - pt_1[1]) / (pt_2[0] - pt_1[0])
            self.b_constant = 1
            self.c_constant = -pt_1[1] - self.a_constant * pt_1[0]

        self.directionality = directionality

    def is_point_in_set(self, new_pt, boundary_inclusive: bool = True) -> bool:
        """
        Checks if the given point is included in the circular set with the option to include the boundary or not.

        Parameters
        ----------
        new_pt
            The (x, y) point to check.
        boundary_inclusive
            Determines if a point on the boundary is included in the set.
            By default, the boundary is part of the set.
        """
        value = self.a_constant * new_pt[0] + self.b_constant * new_pt[1] + self.c_constant

        if not self.directionality:

            return (value <= 0 and boundary_inclusive) or (value < 0 and not boundary_inclusive)

        else:

            return (value >= 0 and boundary_inclusive) or (value > 0 and not boundary_inclusive)

    def calc_point_on_line(self, x_value: float = None, y_value: float = None) -> float:
        """
        Calculate a point on the line given either an x_value or a y_value.
        If both values are given, an error is raised.

        Parameters
        ----------
        x_value
            The x value at which to calculate a corresponding y value.
        y_value
            The y value at which to calculate a corresponding x value.
        """

        # If an x_value is given and a y_value is not, calculate the y_value
        if x_value is not None and y_value is None:
            return ((self.a_constant * x_value) + self.c_constant) / -self.b_constant

        # If a y_value is given and an x_value is not, calculate the x_value
        if x_value is None and y_value is not None:
            return ((self.b_constant * y_value) + self.c_constant) / -self.a_constant

        # Otherwise raise an exception
        else:
            raise Exception("Two values cannot be provided to the function.")


class Circle:

    def __init__(self, origin_pt: tuple, radius: float, directionality: bool = False):

        """
        Creates a circular 2D set defined by an origin point, a radius, and a set directionality.

        False directionality means that points within the boundary are part of the set.

        True directionality means that points outside the boundary are part of the set.

        Parameters
        ----------
        origin_pt
            The center point of the circular set.
        radius
            The radial distance from the origin for the boundary of the set.
        directionality
            Determines which direction that points are considered in the set.
        """

        self.origin_pt = origin_pt

        self.h = -self.origin_pt[0]

        self.k = -self.origin_pt[1]

        self.r_squared = radius ** 2

        self.directionality = directionality

    def is_point_in_set(self, new_pt, boundary_inclusive: bool = True) -> bool:
        """
        Checks if the given point is included in the circular set with the option to include the boundary or not.

        Parameters
        ----------
        new_pt
            The (x, y) point to check.
        boundary_inclusive
            Determines if a point on the boundary is included in the set.
            By default, the boundary is part of the set.
        """

        value = (new_pt[0] + self.h) ** 2 + (new_pt[1] + self.k) ** 2 - self.r_squared

        if not self.directionality:

            return (value <= 0 and boundary_inclusive) or (value < 0 and not boundary_inclusive)

        else:

            return (value >= 0 and boundary_inclusive) or (value > 0 and not boundary_inclusive)


if __name__ == '__main__':

    # Create the node
    node = ImageContactBalanceNode()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    """
    # Uncomment this section to view a single image
    print("---")

    fig = figure("Sector Test", figsize=(6, 6), dpi=300)
    ax = fig.gca()

    # read in an image
    # image = imread(
    #     '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/Test/Images/Series2/Slice_23.png')
    
    node.new_ultrasound_images.append(image)
    
    while len(node.new_ultrasound_images) < 1:
        pass

    image = node.new_ultrasound_images[0]
    image = cvtColor(image, COLOR_GRAY2BGR)

    start_time = time()"""

    while not is_shutdown():
        node.main_loop()

    """
    # Uncomment this section to display one image
    
    display_process_timer(start_time, "Image Translation Time")
    print("---")

    print("Number of points in image: " + str(image.shape[0] * image.shape[1]))
    print("Number of points in sectors: " + str(node.sampled_data_bit_mask.sum()))
    print("Percentage of points in sectors: " +
          str(round(100 * node.sampled_data_bit_mask.sum() / (image.shape[0] * image.shape[1]), 2)) +
          "%")
    print("---")

    gg = 1
    for image_sector in node.sectors:
        print("Sector " + str(gg) + ":")
        print("Avg Value: " + str(image_sector.avg_data_value))
        print("Num Points: " + str(image_sector.num_data_values))
        print("---")
        gg = gg + 1

    ax = imshow(
        node.sampled_data_bit_mask[:, :, newaxis] * image + uint8((1 - node.sampled_data_bit_mask)[:, :, newaxis] * 255)
    )

    plot(
        (node.sectors[0].left_line.pt_1[0], node.sectors[0].left_line.calc_point_on_line(y_value=400)),
        (node.sectors[0].left_line.pt_1[1], 400),
        '-g', linewidth=0.5
    )

    plot(
        (node.sectors[0].right_line.pt_1[0], node.sectors[0].right_line.calc_point_on_line(y_value=400)),
        (node.sectors[0].right_line.pt_1[1], 400),
        '-r', linewidth=0.5
    )

    show()"""
