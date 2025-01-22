"""
File containing the Line class definition.
"""


# Import standard python packages

# Import custom python packages


class Line:

    def __init__(self, first_vertex=None, second_vertex=None,
                 a_constant=None, b_constant=None, c_constant=None):
        """
        Create a LineSegment object. A line segment is a straight line defined by the equation:
        0 = ax + by + c that extends between two vertices.

        Parameters
        ----------
        first_vertex : list or tuple
            a list containing the first (x, y) coordinate defining the line segment.
        second_vertex : list or tuple
            a list containing the second (x, y) coordinate defining the line segment.
        a_constant : float
            a value for the constant a in the equation of the line.
        b_constant : float
            a value for the constant b in the equation of the line.
        c_constant : float
            a value for the constant c in the equation of the line.
        """

        # If two vertices are given,
        if first_vertex is not None and second_vertex is not None and a_constant is None and \
                b_constant is None and c_constant is None:

            # Define variables for ease of reading
            first_x = first_vertex[0]
            second_x = second_vertex[0]
            first_y = first_vertex[1]
            second_y = second_vertex[1]

            # Check if the boundary is vertical
            if second_y - first_y == 0:
                self.a = 0
                self.b = 1
                self.c = -second_y

            # Check if the boundary is horizontal
            elif second_x - first_x == 0:
                self.a = 1
                self.b = 0
                self.c = -second_x

            # Otherwise calculate the slope and y-intercept
            else:
                self.a = -(second_y - first_y) / (second_x - first_x)
                self.b = 1
                self.c = -(first_y + self.a * first_x)

        # If the constants for the line are given,
        elif first_vertex is None and second_vertex is None and a_constant is not None and \
                b_constant is not None and c_constant is not None:

            self.a = a_constant
            self.b = b_constant
            self.c = c_constant

    def is_not_parallel(self, other_line) -> bool:
        """Determines if the given line is parallel to the line."""

        return (self.b != 0 and other_line.b != 0 and (other_line.a / other_line.b) != (
                self.a / self.b)) or \
               (self.b != 0 and other_line.b == 0) or (self.b == 0 and
                                                       other_line.b != 0)

    def calculate_intersection_point(self, other_line) -> tuple:
        """Calculates the intersection point between two lines."""
        return (round((((other_line.b * self.c) - (self.b * other_line.c)) /
                       ((other_line.a * self.b) - (self.a * other_line.b))), 6),
                round((((other_line.c * self.a) - (self.c * other_line.a)) /
                       ((other_line.a * self.b) - (self.a * other_line.b))), 6))

    def distance_to_point(self, point: tuple):
        """Calculates the perpendicular distance from the line to the point"""
        return self.a * point[0] + self.b * point[1] + self.c


if __name__ == '__main__':
    # Create an instance of the object for testing
    test_object = Line()
