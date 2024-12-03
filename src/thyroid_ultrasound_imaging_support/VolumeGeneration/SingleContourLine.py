"""
File containing the CrossContourLine class definition.
"""


# Import standard python packages

# Import custom python packages
from thyroid_ultrasound_imaging_support.VolumeGeneration.Point import Point
from thyroid_ultrasound_imaging_support.VolumeGeneration.CrossContourLine import THIS_CONTOUR, NEXT_CONTOUR


# Define constants for accessing the values in the object
VALUE_DATA: str = 'value_data'
INDEX_DATA: str = 'index_data'


class SingleContourLine:

    def __init__(self, point_1: Point = Point(), point_2: Point = Point(), contour: int = None, line_color: str = None):
        self.point_1: Point = point_1
        self.point_2: Point = point_2
        self.contour: int = contour
        self.color: str = line_color

    def to_shape(self):
        return [(self.point_1.value, self.point_2.value)]

    def get_data_of_point_given_contour(self, contour: int, data_type: str = VALUE_DATA):
        contour = contour % 2
        if contour == self.contour:
            if data_type == VALUE_DATA:
                return self.point_1.value, self.point_2.value
            elif data_type == INDEX_DATA:
                return self.point_1.index, self.point_2.index
            else:
                raise ValueError('A data-type of ' + str(data_type) + ' is not a recognized value.')
        elif contour == THIS_CONTOUR or contour == NEXT_CONTOUR:
            return None, None
        else:
            return ValueError('Contour type is not recognized')

    def add_data_based_on_contour_index(self, contour: int, point_a: Point,
                                        point_b: Point = None):

        contour = contour % 2

        if contour == THIS_CONTOUR or contour == NEXT_CONTOUR:
            self.contour = contour
            self.point_1 = point_a
            self.point_2 = point_b
        else:
            raise ValueError('Contour value not recognized')

        return self

    def get_point_from_contour(self, contour: int):
        contour = contour % 2
        if contour == self.contour:
            return self.point_1, self.point_2
        elif contour == THIS_CONTOUR or contour == NEXT_CONTOUR:
            return None, None
        else:
            raise ValueError('Contour value not recognized')

    def get_vertices(self):
        return [self.point_1.value, self.point_2.value]


if __name__ == '__main__':
    # Create an instance of the object for testing
    test_object = SingleContourLine()
