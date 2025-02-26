"""
Contains the BoundingSet class.
"""
# Import standard packages
from numpy import sign
from thyroid_ultrasound_imaging_support.Boundaries.BoundaryPrimitive import BoundaryPrimitive


class BoundingSet:
    """
    A class to define a bounding set.
    """

    def __init__(self, vertices: list, edge_inclusive=True, interior_inclusive=True):
        """
        Create a BoundingSet object.

        Parameters
        ----------
        vertices
            a list of three or more vertices defining a closed convex polygon.
        edge_inclusive
            defines if a point is included in the set when it is on the edge of the boundary.
            True defines a set that is inclusive of its edges.
        interior_inclusive
            defines whether the set is all points within the bounds or outside the bounds.
            True defines a set that includes the area within the edges.
        """

        self.edge_inclusive = edge_inclusive  # True is inclusive of the edges
        self.interior_inclusive = interior_inclusive  # True means a point inside is in the set
        self.primitives = []

        # Connect each of the vertices with BoundaryPrimitives and add it to the list of primitives.
        # Ensure the BoundaryPrimitives properly wrap around the vertices.
        for index in range(len(vertices)):
            if index == len(vertices) - 1:
                second_index = 0
            else:
                second_index = index + 1

            if second_index == len(vertices) - 1:
                third_index = 0
            else:
                third_index = second_index + 1

            self.primitives.append(
                BoundaryPrimitive(vertices[index], vertices[second_index], vertices[third_index]))

        self.vertices = vertices

    def is_point_within_set(self, point: list):
        """
        Check if a point is contained within the set.

        Parameters
        ----------
        point
            an (x, y) coordinate as a list.
        """

        # For each edge in the bounding set
        for primitive in self.primitives:

            # If the point is not within any singular boundary primitive,
            if not primitive.point_within_boundary(point=point, interior_inclusive=self.interior_inclusive,
                                            edge_inclusive=self.edge_inclusive):
                # Return that the point is not within the set
                return False

            # # Calculate the result of evaluating the boundary line equation at the point
            # result = primitive.a * point[0] + primitive.b * point[1] + primitive.c
            #
            # # Compare the sign of the result above with the sign of the boundary directionality.
            # # If the result is zero, check the inclusivity of the set.
            # if (sign(result) != primitive.directionality and result != 0 and self.interior_inclusive) or (
            #         result == 0 and not self.edge_inclusive):
            #     # Return false if it is not in the set of a single primitive
            #     return False

        # Return true only if the point is in each primitive's set
        return True
