import cv2
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


def get_average_value_from_triangles(list_of_triangles: list, image: np.array):
    lowest_number = 255
    highest_number = 0

    bounding_sets = []

    for triangle in list_of_triangles:
        bounding_sets.append(BoundingSet(triangle))

    for y in range(image.shape[0]):
        for x in range(image.shape[1]):

            for bounding_set in bounding_sets:
                if bounding_set.is_point_within_set((x, y)):
                    a = image[y][x][0]
                    if image[y][x][0] > highest_number:
                        highest_number = image[y][x][0]
                    elif image[y][x][0] < lowest_number:
                        lowest_number = image[y][x][0]
                    break

    return lowest_number, highest_number

def create_mask_array_from_triangles(list_of_background_triangles: list, list_of_foreground_triangles: list,
                                     image_shape: tuple):
    # Create blank mask array and assign it all as probable background
    initial_mask = np.ones(image_shape, np.uint8) * cv2.GC_PR_BGD

    # Define two variables to store the bounding sets within
    background_bounding_sets = []
    foreground_bounding_sets = []

    # Create all background bounding sets from their equivalent triangle
    for background_triangle in list_of_background_triangles:
        background_bounding_sets.append(BoundingSet(background_triangle))

    # Create all foreground bounding sets from their equivalent triangle
    for foreground_triangle in list_of_foreground_triangles:
        foreground_bounding_sets.append(BoundingSet(foreground_triangle))

    # Iterate through each element of the mask
    for y in range(initial_mask.shape[0]):
        for x in range(initial_mask.shape[1]):

            # Add flag to skip checking foreground if the location was in the background
            was_location_background = False

            # Check if the given element is contained within the background, foreground, or neither
            for background_set in background_bounding_sets:
                if background_set.is_point_within_set((x, y)):

                    # Note that the location was found in a boundary
                    was_location_background = True

                    # Update the mask value accordingly
                    initial_mask[y][x] = cv2.GC_BGD

                    # Stop looping once it has been found in one boundary
                    break

            if not was_location_background:
                # Check if the given element is contained within the background, foreground, or neither
                for foreground_set in foreground_bounding_sets:
                    if foreground_set.is_point_within_set((x, y)):

                        # Update the mask value accordingly
                        initial_mask[y][x] = cv2.GC_FGD

                        # Stop looping once it has been found in one boundary
                        break

    # Return the resulting mask
    return initial_mask


class BoundaryPrimitive:

    def __init__(self, first_vertex, second_vertex, reference_vertex, set_directionality=False):
        self.vertices = [first_vertex, second_vertex]
        first_x = first_vertex[0]
        second_x = second_vertex[0]
        first_y = first_vertex[1]
        second_y = second_vertex[1]
        if second_y - first_y == 0:
            self.a_constant = 0
            self.b_constant = 1
            self.c_constant = -second_y
            """if second_x - first_x < 0:
                self.directionality = False # True
            else:
                self.directionality = True # False"""
        elif second_x - first_x == 0:
            self.a_constant = 1
            self.b_constant = 0
            self.c_constant = -second_x
            """if second_y - first_y < 0:
                self.directionality = True # False
            else:
                self.directionality = False # True"""
        else:
            self.a_constant = -(second_y - first_y) / (second_x - first_x)
            self.b_constant = 1
            self.c_constant = -(first_y + self.a_constant * first_x)

            """if self.a_constant < 0 and second_x - first_x < 0:
                self.directionality = True # False
            else:
                self.directionality = False # True"""

        self.directionality = np.sign(self.a_constant * reference_vertex[0] +
                                      self.b_constant * reference_vertex[1] + self.c_constant)

        # 0 = ax + by + c
        # directionality = True = point is in set when primitive is positive
        # directionality = False = point is in set when primitive is negative


class BoundingSet:

    def __init__(self, vertices, set_type=True, directionality=True):

        self.set_type = set_type  # True is inclusive of the edges
        self.set_directionality = directionality  # True means a point inside is in the set
        self.primitives = []

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

    def is_point_within_set(self, point):

        # For each edge in the bounding set
        for primitive in self.primitives:

            # Check to see what the result of the equation for a line is
            result = primitive.a_constant * point[0] + primitive.b_constant * point[1] + primitive.c_constant

            # Check if the point is in the set
            if (np.sign(result) != primitive.directionality and result != 0 and self.set_directionality) or (
                    result == 0 and not self.set_type):

                # Return false if it is not in the set of a single primitive
                return False

        # Return true only if the point is in each primitive's set
        return True


