
# Options to test:
# ----------------
# Checking if the point has already been visited vs not checking if visited
# Checking if the

from cv2 import MORPH_ERODE, MORPH_DILATE, findContours, RETR_EXTERNAL, CHAIN_APPROX_NONE, morphologyEx
from numpy import uint8, zeros, ones
from matplotlib.pyplot import imshow, show, ion
from copy import deepcopy
from time import perf_counter

def custom_morphology(operation, mask, mask_contours, num_pixels):

    if operation == MORPH_DILATE:
        new_value = uint8(1)
    elif operation == MORPH_ERODE:
        new_value = uint8(0)
    else:
        raise Exception('Unrecognized operation type of ' + str(operation) + '.')

    # For each contour
    for contour in mask_contours:

        #   For each point in the contour
        for point in contour:
            point = point[0]

            if operation == MORPH_ERODE and num_pixels is None:
                mask[point[1]][point[0]] = new_value
                # imshow(mask)

            else:

                # Create the ranges of points that would be in the area to change
                new_x_values = range(point[0] - num_pixels, point[0] + num_pixels + 1)
                new_y_values = range(point[1] - num_pixels, point[1] + num_pixels + 1)

                for new_x in new_x_values:
                    for new_y in new_y_values:

                        mask[new_y][new_x] = new_value
                        imshow(mask)

    return mask

if __name__ == '__main__':

    test_size = 1200

    ion()

    test_mask = zeros((12, 12), uint8)

    for y in range(3, 9):
        for x in range(3, 9):
            test_mask[y][x] = uint8(1)

    # second_test_mask = deepcopy(test_mask)

    imshow(test_mask * uint8(255))

    start_time = perf_counter()

    test_contours, hierarchy = findContours(test_mask, RETR_EXTERNAL, CHAIN_APPROX_NONE)

    test_mask = custom_morphology(MORPH_ERODE, test_mask, test_contours, num_pixels=None)

    print(perf_counter() - start_time)

    start_time = perf_counter()

    second_test_mask = morphologyEx(test_mask, MORPH_ERODE, kernel=ones((2, 2)), iterations=1)

    # imshow(second_test_mask * uint8(255))

    print(perf_counter() - start_time)





