#!/usr/bin/env python3

"""
File containing the ImageSector class definition.
"""

# Import standard python packages
from numpy import sin, deg2rad, floor, median, array, append

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageBalancing.LinearSet import LinearSet
from thyroid_ultrasound_imaging_support.ImageBalancing.CircularSet import CircularSet


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
        self.left_line = LinearSet(
            (
                round(image_center + (sides_pixel_to_imaginary_center * sin(deg2rad(left_line_angle)))),
                top_of_ultrasound_image_offset_px
            ),
            (
                image_center, top_of_ultrasound_image_offset_px - sides_pixel_to_imaginary_center
            ),
            left_line_angle,
            left_line_directionality)

        self.right_line = LinearSet(
            (
                round(image_center + (sides_pixel_to_imaginary_center * sin(deg2rad(right_line_angle)))),
                top_of_ultrasound_image_offset_px
            ),
            (
                image_center, top_of_ultrasound_image_offset_px - sides_pixel_to_imaginary_center
            ),
            right_line_angle,
            right_line_directionality)

        # Save the top upper limit of the set
        self.top_edge = top_of_ultrasound_image_offset_px

        # Define the circle used to define the lower edge of the set
        self.bottom_edge_circle = CircularSet(
            (image_center, top_of_ultrasound_image_offset_px - bottom_pixel_to_imaginary_center),
            image_num_rows - top_of_ultrasound_image_offset_px +
            bottom_pixel_to_imaginary_center - bottom_of_ultrasound_image_offset_px)

        # Create variables to store data about the set
        self.avg_data_value = 0.
        self.num_data_values = 0.
        self.is_bright = False

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

    def __add__(self, other):
        """
        Override the add function to allow for easy summing of the average data values.
        """

        if isinstance(other, ImageSector):
            return self.avg_data_value + other.avg_data_value
        elif isinstance(other, (int, float)):
            return self.avg_data_value + other
        else:
            raise Exception(str(type(other)) + " not support for summing")
