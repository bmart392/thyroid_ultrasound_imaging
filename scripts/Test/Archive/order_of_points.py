from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
import matplotlib.pyplot as plt
from copy import deepcopy
from cv2 import circle, line
from numpy.linalg import norm
from numpy import array, nan
from math import isnan

test_object = ImageData(image_data_location='/home/ben/thyroid_ultrasound_data/testing_and_validation/'
                              'saved_image_data_objects/2024-02-13_14-59-07-258616_image-data-object')

fig, axes = plt.subplots(1, 1)

annotation_array = deepcopy(test_object.down_sampled_image)

axes.imshow(annotation_array, cmap='gray')

for contour in test_object.contours_in_image:
    contour = contour[::5]
    for i in range(len(contour)):
        if i == 0:
            prev_point = contour[-1]
        else:
            prev_point = contour[i - 1]
        if i == len(contour) - 1:
            next_point = contour[0]
        else:
            next_point = contour[i + 1]
        this_point = contour[i]
        forward_vector = next_point - this_point
        forward_vector_norm = norm(forward_vector)
        backward_vector = prev_point - this_point
        backward_vector_norm = norm(backward_vector)
        bisector_vector = forward_vector * norm(backward_vector) + backward_vector * norm(forward_vector)
        if sum(bisector_vector) == 0:
            bisector_vector = array([-forward_vector[1], forward_vector[0]])
            bisector_vector = 20 * bisector_vector / norm(bisector_vector)
        else:
            bisector_vector = 20 * bisector_vector / norm(bisector_vector)
        modified_dot_product = (forward_vector[0] * -bisector_vector[1]) + (forward_vector[1] * bisector_vector[0])


        if modified_dot_product > 0:
            bisector_vector = bisector_vector * -1
        bisector_vector = array([round(value) for value in bisector_vector])
        annotation_array = circle(annotation_array, this_point, radius=1, color=(255, 0, 0))
        annotation_array = line(annotation_array, this_point, this_point + bisector_vector, color=(0, 255, 0), thickness=1)
        axes.imshow(annotation_array)
        plt.pause(0.1)
