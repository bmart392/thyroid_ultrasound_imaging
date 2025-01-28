"""
File containing the LineSegment class definition.
"""

# Import standard python packages
from copy import copy

# Import custom python packages
from thyroid_ultrasound_imaging_support.VolumeGeneration.Line import Line


class LineSegment(Line):

    def __init__(self, first_vertex, second_vertex):
        """
        Create a LineSegment object. A line segment is a straight line defined by the equation:
        0 = ax + by + c that extends between two vertices.

        Parameters
        ----------
        first_vertex : list or tuple
            a list containing the first (x, y) coordinate defining the line segment.
        second_vertex : list or tuple
            a list containing the second (x, y) coordinate defining the line segment.
        """

        # Call the super call to calculate the constants for the line
        super().__init__(first_vertex, second_vertex)

        # Save the vertices themselves
        self.vertices = [first_vertex, second_vertex]

    def __eq__(self, other):
        return self.a == other.a and self.b == other.b and self.c == other.c and self.vertices == other.vertices

    def calculate_perpendicular_bisector(self, bisector_location: float = 0.5) -> Line:
        """
        Calculates the constants for a perpendicular bisection line following the equation 0 = ax + by + c

        Parameters
        ----------
        bisector_location :
            The location on the boundary where the bisector should be created as percentage starting from the first vertex
            of the boundary. A given value of 0.5 would place the bisector at the exact middle of the boundary.

        Returns
        -------
        tuple :
            The constants in the order of a, b, c
        """
        # Calculate the point_on_line of this existing_boundary
        point_on_line = ((self.vertices[0][0] * bisector_location) +
                         (self.vertices[1][0] * (1 - bisector_location)),
                         (self.vertices[0][1] * bisector_location) +
                         (self.vertices[1][1] * (1 - bisector_location)))

        # Find the equation of the perpendicular bisector,
        # If the existing_boundary is vertical,
        if self.a == 0:

            # The perpendicular bisector is horizontal,
            perpendicular_bisector_a = 1
            perpendicular_bisector_b = 0
            perpendicular_bisector_c = -point_on_line[0]

        # If the existing_boundary is horizontal,
        elif self.b == 0:

            # The perpendicular bisector is vertical,
            perpendicular_bisector_a = 0
            perpendicular_bisector_b = 1
            perpendicular_bisector_c = -point_on_line[1]

        # Otherwise,
        else:
            perpendicular_bisector_a = -1 / self.a
            perpendicular_bisector_b = 1
            perpendicular_bisector_c = -(point_on_line[1] + perpendicular_bisector_a * point_on_line[0])

        return Line(a_constant=perpendicular_bisector_a, b_constant=perpendicular_bisector_b,
                    c_constant=perpendicular_bisector_c)

    def is_point_on_line_segment(self, point: tuple, edge_inclusive: bool = True):
        """Determines if the given point is on the line."""

        # Define some temporary variables for readability
        x1, x2, y1, y2 = map(float, [min(self.vertices[0][0], self.vertices[1][0]),
                                     max(self.vertices[0][0], self.vertices[1][0]),
                                     min(self.vertices[0][1], self.vertices[1][1]),
                                     max(self.vertices[0][1], self.vertices[1][1])])
        if edge_inclusive:
            return x1 <= point[0] <= x2 and y1 <= point[1] <= y2
        else:
            return (x1 < point[0] < x2 and y1 < point[1] < y2) or \
                   (x1 == point[0] and y1 < point[1] < y2) or \
                   (x1 < point[0] < x2 and y1 == point[1])

    def check_if_boundary_is_external(self, all_boundaries, bisector_locations: tuple = (0.001, 0.5, 0.999)):

        # Create a copy of the complete list of boundaries
        all_other_boundaries = copy(all_boundaries)

        # Pop out the boundary from the boundaries to check if is in the list of other boundaries
        try:
            all_other_boundaries.pop(all_boundaries.index(self))

        except ValueError:
            pass

        # Create a list to store the bisectors that will be used to check the if the boundary is external
        bisectors = []

        # Create a bisector for each location,
        for location in bisector_locations:
            bisectors.append(self.calculate_perpendicular_bisector(location))

        # Define two variables to track how many boundaries were found to intercept the perpendicular bisector
        intersecting_boundaries_within_boundary = 0
        intersecting_boundaries_outside_boundary = 0

        # For every other existing_boundary,
        for boundary in all_other_boundaries:

            # First check if the other boundary is parallel to this boundary itself
            if self.is_not_parallel(boundary):

                # Find the intersection point between the two boundaries
                boundary_intersection_point = self.calculate_intersection_point(boundary)

                # If the intersection point between the two boundaries is on both boundaries,
                if (self.is_point_on_line_segment(boundary_intersection_point, edge_inclusive=False) and
                        boundary.is_point_on_line_segment(boundary_intersection_point, edge_inclusive=False)):

                    # Then the boundary cannot be external because it intersects with another boundary
                    return False

            # If the two lines are not parallel,
            if bisectors[0].is_not_parallel(boundary):

                # For each bisector,
                for single_bisector in bisectors:

                    # Calculate the intersection point between the bisector and the boundary
                    intersection_point = single_bisector.calculate_intersection_point(boundary)

                    # If the intersection point is actually on the boundary,
                    if boundary.is_point_on_line_segment(intersection_point):

                        # If the distance to the intersection point is negative,
                        if self.distance_to_point(intersection_point) < 0:
                            # Increment the number of boundaries that intersect the bisector on the negative side
                            intersecting_boundaries_within_boundary = intersecting_boundaries_within_boundary + 1
                            break

                        # Elif the distance to the intersection point is positive,
                        elif self.distance_to_point(intersection_point) > 0:
                            # Increment the number of boundaries that intersect the bisector on the positive side
                            intersecting_boundaries_outside_boundary = intersecting_boundaries_outside_boundary + 1
                            break

        # If the number of positive side intersections != the number of negative side intersections and
        # one of them equals 0,
        return (intersecting_boundaries_within_boundary != intersecting_boundaries_outside_boundary and
                (intersecting_boundaries_within_boundary == 0 or intersecting_boundaries_outside_boundary == 0))

    def get_vertices_as_x_and_y(self):
        return [self.vertices[0][0], self.vertices[1][0]], [self.vertices[0][1], self.vertices[1][1]]


if __name__ == '__main__':
    # Create an instance of the object for testing
    test_object = LineSegment((0, 0), (1, 1))
