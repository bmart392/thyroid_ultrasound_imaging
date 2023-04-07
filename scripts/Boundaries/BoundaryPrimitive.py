"""
Contains the BoundaryPrimitive class.
"""
# Import standard packages
from numpy import sign


class BoundaryPrimitive:
    """
    A class to define a boundary primitive.
    """

    def __init__(self, first_vertex: list, second_vertex: list, reference_vertex: list):
        """
        Create a BoundaryPrimitive object. A boundary is a straight line defined by the equation:
        0 = ax + by + c.

        Parameters
        ----------
        first_vertex
            a list containing the first (x, y) coordinate defining the boundary line.
        second_vertex
            a list containing the second (x, y) coordinate defining the boundary line.
        reference_vertex
            a list containing a third (x, y) coordinate that is known to be on the side of
            the boundary that is within the set.
        """

        # Save the vertices that define the boundary as a list
        self.vertices = [first_vertex, second_vertex]

        # Define variables for ease of reading
        first_x = first_vertex[0]
        second_x = second_vertex[0]
        first_y = first_vertex[1]
        second_y = second_vertex[1]

        # Check if the boundary is vertical
        if second_y - first_y == 0:
            self.a_constant = 0
            self.b_constant = 1
            self.c_constant = -second_y

        # Check if the boundary is horizontal
        elif second_x - first_x == 0:
            self.a_constant = 1
            self.b_constant = 0
            self.c_constant = -second_x

        # Otherwise calculate the slope and y-intercept
        else:
            self.a_constant = -(second_y - first_y) / (second_x - first_x)
            self.b_constant = 1
            self.c_constant = -(first_y + self.a_constant * first_x)

        # Calculate the result of evaluating the boundary equation at the reference point.
        # The sign of this result is the directionality.
        self.directionality = sign(self.a_constant * reference_vertex[0] +
                                   self.b_constant * reference_vertex[1] + self.c_constant)
