import numpy as np

RECTANGLE_2_PT_CORNER: int = 1
RECTANGLE_2_PT_CENTER: int = 2
RECTANGLE_3_PT_CORNER: int = 3
RECTANGLE_3_PT_CENTER: int = 4
RECTANGLE_3_PT_PARALLELOGRAM: int = 5

CIRCLE_1_PT_RADIUS: int = 10
CIRCLE_3_PT: int = 11

ELLIPSE_2_PT: int = 15

SLOT_STRAIGHT_2_PT_END_RADIUS: int = 20
SLOT_STRAIGHT_2_PT_CENTER_RADIUS: int = 21
SLOT_ARC_3_PT_RADIUS: int = 22


def draw_rectangle(rectangle_type, coordinates, width_x, height_y, value, array_to_modify):
    if rectangle_type == RECTANGLE_2_PT_CORNER:
        boundaries_x = (coordinates[0][0], coordinates[1][0])
        boundaries_y = (coordinates[0][1], coordinates[1][1])
    elif rectangle_type == "center":
        boundaries_x = (round(coordinates[0] - width_x / 2), round(coordinates[0] + width_x / 2))
        boundaries_y = (round(coordinates[1] - height_y / 2), round(coordinates[1] + height_y / 2))
    else:
        return

    for x in range(boundaries_x[0], boundaries_x[1]):
        for y in range(boundaries_y[0], boundaries_y[1]):
            array_to_modify[y][x] = value

    return array_to_modify


def draw_circle(origin, radius, value, array_to_modify):
    boundaries_x = (origin[0] - radius, origin[0] + radius)
    boundaries_y = (origin[1] - radius, origin[1] + radius)

    for x in range(boundaries_x[0], boundaries_x[1]):
        for y in range(boundaries_y[0], boundaries_y[1]):
            distance_to_center = (abs(x - origin[0]) ** 2 + abs(y - origin[1]) ** 2) ** .5
            if distance_to_center < radius:
                array_to_modify[y][x] = value

    return array_to_modify


class LinePrimitive:

    def __init__(self, first_vertex, second_vertex):
        self.vertices = [first_vertex, second_vertex]
        first_x = first_vertex[0]
        second_x = second_vertex[0]
        first_y = first_vertex[1]
        second_y = second_vertex[1]
        if second_y - first_y == 0:
            self.a_constant = 0
            self.b_constant = 1
            self.c_constant = -second_y
            if second_x - first_x < 0:
                self.directionality = True
            else:
                self.directionality = False
        elif second_x - first_x == 0:
            self.a_constant = 1
            self.b_constant = 0
            self.c_constant = -second_x
            if second_y - first_y < 0:
                self.directionality = False
            else:
                self.directionality = True
        else:
            self.a_constant = -(second_vertex[1] - first_vertex[1]) / (second_vertex[0] - first_vertex[0])
            self.b_constant = 1
            self.c_constant = first_vertex[1] + self.a_constant * first_vertex[0]

            if self.a_constant < 0 and second_x - first_x < 0:
                self.directionality = False
            else:
                self.directionality = True

        # 0 = ax + by + c
        # directionality = True = point is in set when primitive is positive
        # directionality = False = point is in set when primitive is negative


class CirclePrimitive:

    def __init__(self, circle_type: int, circle_parameters: list, directionality: bool = False):

        self.directionality = directionality

        if circle_type == CIRCLE_1_PT_RADIUS:
            self.a_constant = circle_parameters[0][0]
            self.b_constant = circle_parameters[0][1]
            self.radius = circle_parameters[1]

            self.directionality = directionality

        if circle_type == CIRCLE_3_PT:
            if len(circle_parameters) == 3:
                point_0 = circle_parameters[0]
                point_1 = circle_parameters[1]
                point_2 = circle_parameters[2]
            else:
                raise Exception("Incorrect number of parameters given.")

            determinant_minor_1 = np.linalg.det(np.array([
                [point_0[0], point_0[1], 1],
                [point_1[0], point_1[1], 1],
                [point_2[0], point_2[1], 1],
            ]))

            if determinant_minor_1 == 0:
                raise Exception("Circle cannot be constructed.")

            determinant_minor_2 = np.linalg.det(np.array([
                [point_0[0] ** 2 + point_0[1] ** 2, point_0[1], 1],
                [point_1[0] ** 2 + point_1[1] ** 2, point_1[1], 1],
                [point_2[0] ** 2 + point_2[1] ** 2, point_2[1], 1],
            ]))

            determinant_minor_3 = np.linalg.det(np.array([
                [point_0[0] ** 2 + point_0[1] ** 2, point_0[0], 1],
                [point_1[0] ** 2 + point_1[1] ** 2, point_1[0], 1],
                [point_2[0] ** 2 + point_2[1] ** 2, point_2[0], 1],
            ]))

            determinant_minor_4 = np.linalg.det(np.array([
                [point_0[0] ** 2 + point_0[1] ** 2, point_0[0], point_0[1]],
                [point_1[0] ** 2 + point_1[1] ** 2, point_1[0], point_1[1]],
                [point_2[0] ** 2 + point_2[1] ** 2, point_2[0], point_2[1]],
            ]))

            self.a_constant = 0.5 * determinant_minor_2 / determinant_minor_1
            self.b_constant = -0.5 * determinant_minor_3 / determinant_minor_1
            self.radius = (self.a_constant ** 2 + self.b_constant ** 2 +
                           determinant_minor_4 / determinant_minor_1) ** 0.5
