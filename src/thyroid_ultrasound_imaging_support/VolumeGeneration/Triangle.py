"""
File containing the Triangle class definition.
"""


# Import standard python packages

# Import custom python packages
from thyroid_ultrasound_imaging_support.VolumeGeneration.CrossContourLine import CrossContourLine
from thyroid_ultrasound_imaging_support.VolumeGeneration.SingleContourLine import SingleContourLine


class Triangle:

    def __init__(self, cross_contour_line_a: CrossContourLine = CrossContourLine(),
                 cross_contour_line_b: CrossContourLine = CrossContourLine(),
                 single_contour_line: SingleContourLine = SingleContourLine()):
        self.cross_contour_line_a = cross_contour_line_a
        self.cross_contour_line_b = cross_contour_line_b
        self.single_contour_line = single_contour_line

    def get_constituent_shapes(self):
        return [self.cross_contour_line_a, self.cross_contour_line_b, self.single_contour_line]

    def convert_to_vertices(self):
        vertices = []

        for shape in self.get_constituent_shapes():
            for vertex in shape.get_vertices():
                if vertex not in vertices:
                    vertices.append(vertex)

        return tuple(vertices)


if __name__ == '__main__':
    # Create an instance of the object for testing
    test_object = Triangle()
