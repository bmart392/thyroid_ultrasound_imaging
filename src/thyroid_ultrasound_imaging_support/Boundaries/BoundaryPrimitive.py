"""
Contains the BoundaryPrimitive class.
"""
# Import standard packages
from numpy import sign


class BoundaryPrimitive:
    """
    A class to define a boundary primitive.
    """

    def __init__(self, first_vertex, second_vertex, reference_vertex = None):
        """
        Create a BoundaryPrimitive object. A boundary is a straight line defined by the equation:
        0 = ax + by + c.

        Parameters
        ----------
        first_vertex : list or tuple
            a list containing the first (x, y) coordinate defining the boundary line.
        second_vertex : list or tuple
            a list containing the second (x, y) coordinate defining the boundary line.
        reference_vertex : list or tuple
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

        # If a reference point is given,
        if reference_vertex is not None:
            # Calculate the result of evaluating the boundary equation at the reference point.
            # The sign of this result is the directionality.
            self.directionality = sign(self.a_constant * reference_vertex[0] +
                                       self.b_constant * reference_vertex[1] + self.c_constant)
        # Otherwise, arbitrarily assign it
        else:
            self.directionality = -1

    def point_within_boundary(self, point, interior_inclusive: bool = True,
                              edge_inclusive: bool = True, throw_edge_exception: bool = False) -> bool:
        """
        Determines if the given point falls within the boundary.

        Parameters
        ----------
        point : tuple or list
            A 2D point as a tuple
        interior_inclusive :
            Allows the boundary to be flipped such that points outside the boundary are detected
        edge_inclusive :
            Allows points on the edge of the boundary to not be counted as part of the set
        Returns
        -------
        bool :
            Whether the point is in the set or not
        """

        # Calculate the result of evaluating the boundary line equation at the point
        result = self.a_constant * point[0] + self.b_constant * point[1] + self.c_constant

        if throw_edge_exception and result == 0:
            raise EdgeException(str(point) + ' lies on the boundary.')

        # Compare the sign of the result above with the sign of the boundary directionality.
        # If the result is zero, check the inclusivity of the set.
        return (sign(result) == self.directionality and result != 0 and interior_inclusive) or \
               (sign(result) != self.directionality and result != 0 and not interior_inclusive) or \
               (result == 0 and edge_inclusive)


class EdgeException(Exception):

    def __init__(self, message):
        self.message = message
        super().__init__(self.message)
