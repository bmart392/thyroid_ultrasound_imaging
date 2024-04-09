
from cv2 import imread, imshow, waitKey, threshold, THRESH_BINARY, \
    morphologyEx, MORPH_CLOSE, MORPH_OPEN, \
    findContours, RETR_EXTERNAL, CHAIN_APPROX_NONE, \
    moments
from numpy import uint8, std, ones, array
from statistics import median, stdev, mean

from thyroid_ultrasound_imaging_support.UserInput.display_image_with_callback import display_image_with_callback


def find_closest_point(reference_coordinate, list_of_coordinate_pairs):
    possible_pairs = []
    for coordinate_pair in list_of_coordinate_pairs:
        if abs(coordinate_pair[0] - reference_coordinate[0]) < 0.1 * reference_coordinate[0]:
            possible_pairs.append(coordinate_pair)
    if len(possible_pairs) < 1:
        raise KeyError('No matching X coordinate could be found.')
    for coordinate_pair in possible_pairs:
        if abs(coordinate_pair[1] - reference_coordinate[1]) < 0.1 * reference_coordinate[1]:
            list_of_coordinate_pairs.pop(list_of_coordinate_pairs.index(coordinate_pair))
            return tuple(coordinate_pair)
    raise KeyError('No matching Y coordinate could be found.')


base_image_path = '/home/ben/thyroid_ultrasound_data/testing_and_validation/calibration_data'
img_paths = ['/depth_52/52_0.JPEG', '/depth_52/52_1.JPEG', '/depth_52/52_2.JPEG', '/depth_52/52_3.JPEG',
             '/depth_52/52_4.JPEG', '/depth_52/52_5.JPEG', '/depth_52/52_6.JPEG', '/depth_52/52_7.JPEG']
crop_coordinate_pairs = [[(246, 303), (478, 498)], [(235, 350), (449, 517)], [(315, 371), (535, 521)],
                    [(293, 338), (531, 482)], [(311, 327), (548, 493)], [(301, 305), (545, 480)],
                    [(337, 317), (590, 465)], [(223, 325), (474, 487)]]

# Define a place to store the pixel distances calculated
vertical_distances = []
horizontal_distances = []

for img_path, crop_coordinate_pair in zip(img_paths, crop_coordinate_pairs):
    print('-' * 27)
    print(('-' * 3) + ' ' + img_path + ' ' + ('-' * 3))
    print('-' * 27)

    # Load the image
    img = imread(base_image_path + img_path)

    # Select the points around the dots
    if crop_coordinate_pair is None:
        first_pt = display_image_with_callback(img, 'Select the upper left corner', [0, 0])
        second_pt = display_image_with_callback(img, 'Select the upper right corner', [0, 0])
    else:
        first_pt = crop_coordinate_pair[0]
        second_pt = crop_coordinate_pair[1]
    print('First pt: ' + str(first_pt))
    print('Second pt: ' + str(second_pt))


    # Save a copy of the cropped image
    cropped_image_colored = img[first_pt[1]:second_pt[1], first_pt[0]:second_pt[0], :]
    cropped_image = cropped_image_colored[:, :, 0]

    # Find the mean and standard deviation of the array to find the threshold
    mean_intensity = mean(cropped_image.flatten())
    std_deviation = std(cropped_image.flatten())

    # Segment the images to find the bright areas
    _, cropped_image_mask = threshold(cropped_image, mean_intensity + 2.0 * std_deviation, 1, THRESH_BINARY)

    # Remove noise and ensure that each contour is continuous
    cropped_image_mask = morphologyEx(cropped_image_mask, MORPH_CLOSE, ones((5, 5), dtype=uint8))
    cropped_image_mask = morphologyEx(cropped_image_mask, MORPH_OPEN, ones((5, 5), dtype=uint8))

    # Find the contours in the image
    contours_in_image, hierarchy = findContours(
                    cropped_image_mask,
                    RETR_EXTERNAL,
                    CHAIN_APPROX_NONE
                )

    # Find the centroid of each contour
    contour_centroids = []
    for contour in contours_in_image:
        temp_moments = moments(array(contour))
        temp_centroid_x = int(temp_moments["m10"] / temp_moments["m00"])
        temp_centroid_y = int(temp_moments["m01"] / temp_moments["m00"])
        contour_centroids.append([temp_centroid_x, temp_centroid_y])

    # Find the 3 valid X positions
    sorted_x = sorted([x[0] for x in contour_centroids])
    independent_x_values = [sorted_x[0]]
    independent_j = 0
    for i in range(1, len(sorted_x)):
        if sorted_x[i] > 1.15 * independent_x_values[independent_j]:
            independent_x_values.append(sorted_x[i])
            independent_j = independent_j + 1

    # Find the 4 valid Y positions
    sorted_y = sorted([x[1] for x in contour_centroids])
    independent_y_values = [sorted_y[0]]
    independent_j = 0
    for i in range(1, len(sorted_y)):
        if sorted_y[i] > 1.15 * independent_y_values[independent_j]:
            independent_y_values.append(sorted_y[i])
            independent_j = independent_j + 1

    # Define a place to store the coordinates in each row
    rows = [[], [], [], []]

    # Find the coordinates in each row
    for y, result_array in zip(independent_y_values, rows):
        for x in independent_x_values:
            try:
                result_array.append(find_closest_point((x, y), contour_centroids))
            except KeyError:
                result_array.append(None)

    # Find the distances available from the points
    for y in range(len(rows)):
        for x in range(len(rows[y])):
            if rows[y][x] is not None:
                if x > 0:
                    horizontal_distances.append(rows[y][x][0] - rows[y][x - 1][0])
                if y > 0:
                    if rows[y - 1][x] is not None:
                        vertical_distances.append(rows[y][x][1] - rows[y - 1][x][1])

HORIZONTAL = 'Horizontal - '
VERTICAL = 'Vertical --- '
PX = '   (px): '
print('-' * 27)
print(('-' * 9) + ' Summary ' + ('-' * 9))
print('-' * 27)
print(HORIZONTAL + 'Min    ' + PX + str(round(min(horizontal_distances), 2)))
print(HORIZONTAL + 'Max    ' + PX + str(round(max(horizontal_distances), 2)))
print(HORIZONTAL + 'Mean   ' + PX + str(round(mean(horizontal_distances), 4)))
print(HORIZONTAL + 'Median ' + PX + str(round(median(horizontal_distances), 4)))
print(HORIZONTAL + 'Std Dev' + PX + str(round(stdev(horizontal_distances), 4)))
print(HORIZONTAL + 'Resolution (m): ' + str(round(0.01 / mean(horizontal_distances), 8)))
print('-' * 27)
print(VERTICAL + 'Min    ' + PX + str(round(min(vertical_distances), 2)))
print(VERTICAL + 'Max    ' + PX + str(round(max(vertical_distances), 2)))
print(VERTICAL + 'Mean   ' + PX + str(round(mean(vertical_distances), 4)))
print(VERTICAL + 'Median ' + PX + str(round(median(vertical_distances), 4)))
print(VERTICAL + 'Std Dev' + PX + str(round(stdev(vertical_distances), 4)))
print(VERTICAL + 'Resolution (m): ' + str(round(0.005 / mean(vertical_distances), 8)))



