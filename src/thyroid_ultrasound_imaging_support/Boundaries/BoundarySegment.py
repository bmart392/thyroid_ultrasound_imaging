"""
File containing the BoundarySegment class definition.
"""


# Import standard python packages
from typing import List

# Import custom python packages
from thyroid_ultrasound_imaging_support.Boundaries.BoundaryPrimitive import BoundaryPrimitive, EdgeException
from thyroid_ultrasound_imaging_support.VolumeGeneration.LineSegment import LineSegment


class BoundarySegment:

    def __init__(self, list_of_boundary_primitives: List[LineSegment] = None,
                 prior_boundary_primitive_external: bool = False,
                 next_boundary_primitive_external: bool = False):

        self.boundaries_contained_within = list_of_boundary_primitives

        if self.boundaries_contained_within is not None and isinstance(self.boundaries_contained_within, list):

            self.starting_vertex = self.boundaries_contained_within[0].vertices[0]
            self.ending_vertex = self.boundaries_contained_within[-1].vertices[1]
            self.length = len(self.boundaries_contained_within)

        else:
            self.starting_vertex = None
            self.ending_vertex = None
            self.length = 0

        self.prior_boundary_primitive_external = prior_boundary_primitive_external
        self.next_boundary_primitive_external = next_boundary_primitive_external

    def get_all_vertices(self):
        result_list = [self.starting_vertex]
        for primitive in self.boundaries_contained_within:
            result_list.append(primitive.vertices[-1])
        return result_list

if __name__ == '__main__':
    # Create an instance of the object for testing
    test_object = BoundarySegment()
