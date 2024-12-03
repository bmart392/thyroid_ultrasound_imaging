"""
File containing the CrossContourLine class definition.
"""


# Import standard python packages

# Import custom python packages
from thyroid_ultrasound_imaging_support.VolumeGeneration.Point import Point


# Define constants for accessing the values in the object
VALUE_DATA: str = 'value_data'
INDEX_DATA: str = 'index_data'

# Define constants useful for indexing
THIS_CONTOUR: int = 0
NEXT_CONTOUR: int = 1


class CrossContourLine:

    def __init__(self, point_1: Point = Point(), point_2: Point = Point(), line_color: str = None):
        self.point_1: Point = point_1
        self.point_2: Point = point_2
        self.color: str = line_color

    def to_shape(self):
        return [(self.point_1.value, self.point_2.value)]

    def get_data_of_point_given_contour(self, contour: int, data_type: str = VALUE_DATA):
        contour = contour % 2
        if contour == THIS_CONTOUR:
            if data_type == VALUE_DATA:
                return self.point_1.value
            elif data_type == INDEX_DATA:
                return self.point_1.index
            else:
                raise ValueError('A data-type of ' + str(data_type) + ' is not a recognized value.')
        if contour == NEXT_CONTOUR:
            if data_type == VALUE_DATA:
                return self.point_2.value
            elif data_type == INDEX_DATA:
                return self.point_2.index
            else:
                raise ValueError('A data-type of ' + str(data_type) + ' is not a recognized value.')

    def add_data_based_on_contour_index(self, contour_a: int, point_a: Point,
                                        contour_b: int = None, point_b: Point = None):

        contour_a = contour_a % 2
        if contour_a == THIS_CONTOUR:
            self.point_1 = point_a
        elif contour_a == NEXT_CONTOUR:
            self.point_2 = point_a
        else:
            raise ValueError('Contour value not recognized')

        if contour_b is not None and contour_a != contour_b:
            contour_b = contour_b % 2
            if contour_b == THIS_CONTOUR:
                self.point_1 = point_b
            elif contour_b == NEXT_CONTOUR:
                self.point_2 = point_b
            else:
                raise ValueError('Contour value not recognized')

        return self

    def get_point_from_contour(self, contour: int):
        contour = contour % 2
        if contour == THIS_CONTOUR:
            return self.point_1
        elif contour == NEXT_CONTOUR:
            return self.point_2
        else:
            raise ValueError('Contour value not recognized')

    def get_vertices(self):
        return [self.point_1.value, self.point_2.value]

if __name__ == '__main__':
    # Create an instance of the object for testing
    test_object = CrossContourLine()
