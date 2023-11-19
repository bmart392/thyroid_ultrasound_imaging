#!/usr/bin/env python3

"""
File containing the CircularSet class definition.
"""


class CircularSet:

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