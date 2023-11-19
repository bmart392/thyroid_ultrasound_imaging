#!/usr/bin/env python3

"""
File containing the LinearSet class definition.
"""


class LinearSet:

    def __init__(self, pt_1: tuple, pt_2: tuple, line_angle: float = None, directionality: bool = False):
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
        line_angle:
            The angle used to generate the line.
        directionality:
            Determines which direction that points are considered in the set.
        """

        # Save the points used to generate the line
        self.pt_1 = pt_1
        self.pt_2 = pt_2

        # Save the angle used to generate the line
        self.line_angle = line_angle

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
