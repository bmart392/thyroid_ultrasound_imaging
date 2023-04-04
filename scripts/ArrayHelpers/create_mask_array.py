from enum import Enum
import numpy as np
import cv2


class ShapeTypes(Enum):
    RECTANGLE, CIRCLE = range(2)


class RectangleTypes(Enum):
    FLAT_CORNER, FLAT_CORNER_SQUARE, FLAT_CENTER, ANGLED_CORNER, ANGLED_CORNER_SQUARE, ANGLED_CENTER = range(6)


def draw_shape(shape_type, origin_coordinate, value, array_to_modify, rect_type=None, radius=None, width=None, height=None):
    boundaries_x = (0, 0)
    boundaries_y = (0, 0)
    if shape_type == ShapeTypes.CIRCLE:
        boundaries_x = (origin_coordinate[0] - radius, origin_coordinate[0] + radius)
        boundaries_y = (origin_coordinate[1] - radius, origin_coordinate[1] + radius)
    elif shape_type == ShapeTypes.RECTANGLE:
        if rect_type is None:
            raise Exception("Rectangle type not specified.")

        if rect_type == RectangleTypes.FLAT_CORNER:
            boundaries_x = (origin_coordinate[0], origin_coordinate[0] + width)
            boundaries_y = (origin_coordinate[1], origin_coordinate[1] + height)
        if rect_type == RectangleTypes.FLAT_CENTER:
            boundaries_x = (origin_coordinate[0] - width, origin_coordinate[1] + width)
            boundaries_y = (origin_coordinate[1] - height, origin_coordinate[1] + height)
        if rect_type == RectangleTypes.FLAT_CORNER_SQUARE:
            if (width is None and height is None) or (width is not None and height is not None and width != height):
                raise Exception("Either width of height must be specified. If both are specified they must be the same.")
            if width is None:
                dimension = height
            else:
                dimension = width
            boundaries_x = (origin_coordinate[0], origin_coordinate[0] + dimension)
            boundaries_y = (origin_coordinate[1], origin_coordinate[1] + dimension)

    else:
        raise Exception("Shape type provided was not recognized.")

    for x in range(boundaries_x[0], boundaries_x[1]):
        for y in range(boundaries_y[0], boundaries_y[1]):

            if shape_type == ShapeTypes.RECTANGLE and (rect_type == RectangleTypes.FLAT_CORNER or
                                                       rect_type == RectangleTypes.FLAT_CENTER or
                                                       rect_type == RectangleTypes.FLAT_CORNER_SQUARE):
                array_to_modify[y][x] = value
            else:
                fill_in_condition_met = False

                if shape_type == ShapeTypes.CIRCLE:
                    distance_to_center = ((x - origin_coordinate[0]) ** 2 + (y - origin_coordinate[1]) ** 2) ** .5
                    fill_in_condition_met = distance_to_center < radius

                if fill_in_condition_met:
                    array_to_modify[y][x] = value

    return array_to_modify


def create_mask_array(mask_shapes: list, image_shape) -> np.array:
    initialized_mask = np.ones(image_shape, np.uint8) * cv2.GC_PR_BGD
    for shape in mask_shapes:
        initialized_mask = draw_shape(shape_type=shape[0],
                                      rect_type=shape[1],
                                      origin_coordinate=(shape[2], shape[3]),
                                      width=shape[4],
                                      height=shape[5],
                                      radius=shape[6],
                                      value=shape[8],
                                      array_to_modify=initialized_mask)
    return initialized_mask
