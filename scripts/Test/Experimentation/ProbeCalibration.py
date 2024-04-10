from cv2 import imread, imshow, waitKey, destroyAllWindows, destroyWindow, \
    cvtColor, COLOR_GRAY2BGR, \
    threshold, THRESH_BINARY, \
    morphologyEx, MORPH_CLOSE, MORPH_OPEN, \
    findContours, RETR_EXTERNAL, CHAIN_APPROX_NONE, \
    moments, \
    circle
from numpy import uint8, std, ones, array, append
from numpy.linalg import norm
from statistics import median, stdev, mean
from copy import deepcopy

from thyroid_ultrasound_imaging_support.UserInput.display_image_with_callback import display_image_with_callback


# ----------------------------------------------------------------------------------------------------------------------

def find_closest_point(reference_coordinate: tuple, list_of_coordinate_pairs: list,
                       tolerance: float = 0.1, use_norm: bool = False) -> tuple:
    """
    Finds a coordinate that is equivalent to the reference coordinate from within the list of coordinate pairs.

    Parameters
    ----------
    reference_coordinate :
        The coordinate to find within the list.
    list_of_coordinate_pairs :
        The list of coordinates in which to search.
    tolerance :
        The threshold for which values are considered equivalent as a percentage of the reference value.
    use_norm :
        Commands the function to find the closest centroid using the euclidean distance.

    Returns
    -------
    tuple
        The coordinate pair that matches the reference coordinate within the given tolerance
    """

    if use_norm:
        closest_coordinate_distance = 0
        closest_coordinate = None
        for coordinate in list_of_coordinate_pairs:
            new_distance = norm(array(coordinate) - array(reference_coordinate))
            if new_distance < closest_coordinate_distance or \
                    closest_coordinate is None:
                closest_coordinate_distance = new_distance
                closest_coordinate = coordinate
        if closest_coordinate is None:
            raise Exception('Closest coordinate could not be found.')
        else:
            list_of_coordinate_pairs.pop(list_of_coordinate_pairs.index(closest_coordinate))
            return tuple(closest_coordinate)

    else:
        # Define a list to store the points which may be paired with the reference point based on the X value
        possible_pairs = []

        # For each coordinate, compare the X values and if they are similar add them to the list
        for coordinate_pair in list_of_coordinate_pairs:
            if abs(coordinate_pair[0] - reference_coordinate[0]) < tolerance * reference_coordinate[0]:
                possible_pairs.append(coordinate_pair)

        # Raise an error if no matching pair could be found
        if len(possible_pairs) < 1:
            raise KeyError('No matching X coordinate could be found.')

        # For each possible pair, compare the Y values to find the matching coordinate
        for coordinate_pair in possible_pairs:
            if abs(coordinate_pair[1] - reference_coordinate[1]) < tolerance * reference_coordinate[1]:
                list_of_coordinate_pairs.pop(list_of_coordinate_pairs.index(coordinate_pair))
                return tuple(coordinate_pair)

        # Otherwise raise an error.
        raise KeyError('No matching Y coordinate could be found.')


# ----------------------------------------------------------------------------------------------------------------------
class DataSet:
    def __init__(self, name: str, paths: list, crop_coordinates: list, centroids_to_ignore: dict,
                 imaging_depth_meters: float, segmentation_verified: bool):
        self.name = name
        self.paths = paths
        self.crop_coordinates = crop_coordinates
        self.centroids_to_ignore = centroids_to_ignore
        self.imaging_depth_meters = imaging_depth_meters
        self.segmentation_verified = segmentation_verified


# ----------------------------------------------------------------------------------------------------------------------
# Define the distances within the phantom
HORIZONTAL_PHANTOM_DISTANCE: float = 0.01  # 10 mm
VERTICAL_PHANTOM_DISTANCE: float = 0.005  # 5 mm

# ----------------------------------------------------------------------------------------------------------------------

# Select if the key values from the keyboard need to be verified
verify_key_values = False

base_image_path = '/home/ben/thyroid_ultrasound_data/testing_and_validation/calibration_data'

depth45_data = DataSet(name='4.5cm Depth',
                       paths=['/depth_45/45_0.JPEG', '/depth_45/45_1.JPEG', '/depth_45/45_2.JPEG',
                              '/depth_45/45_3.JPEG', '/depth_45/45_4.JPEG', '/depth_45/45_5.JPEG',
                              '/depth_45/45_6.JPEG'],
                       crop_coordinates=[[(310, 301), (583, 488)], [(358, 304), (634, 488)], [(168, 297), (469, 503)],
                                         [(168, 272), (481, 471)], [(248, 270), (527, 454)], [(236, 320), (523, 510)],
                                         [(291, 337), (575, 530)]],
                       centroids_to_ignore={'/depth_45/45_0.JPEG': [(196, 19)],
                                            '/depth_45/45_1.JPEG': [(181, 22), (190, 9), (177, 8)],
                                            '/depth_45/45_2.JPEG': [(230, 8), (227, 22), (228, 35)],
                                            '/depth_45/45_3.JPEG': [(46, 69), (233, 21)],
                                            '/depth_45/45_6.JPEG': [(196, 37), (199, 19)]},
                       imaging_depth_meters=0.045,
                       segmentation_verified=True)

depth50_data = DataSet(name='5.0cm Depth',
                       paths=['/depth_50/50_1.JPEG', '/depth_50/50_2.JPEG', '/depth_50/50_3.JPEG',
                              '/depth_50/50_4.JPEG', '/depth_50/50_5.JPEG'],
                       crop_coordinates=[[(303, 390), (539, 554)], [(311, 336), (546, 512)],
                                         [(228, 338), (453, 495)], [(360, 346), (594, 510)]],
                       centroids_to_ignore={'/depth_50/50_1.JPEG': [(167, 55), (190, 54), (184, 138)],
                                            '/depth_50/50_2.JPEG': [(163, 66), (169, 54)],
                                            '/depth_50/50_3.JPEG': [(171, 68), (179, 55), (176, 154),
                                                                    (182, 141), (171, 141)],
                                            },
                       imaging_depth_meters=0.050,
                       segmentation_verified=True)

depth52_data = DataSet(name='5.2cm Depth',
                       paths=['/depth_52/52_0.JPEG', '/depth_52/52_1.JPEG', '/depth_52/52_2.JPEG',
                              '/depth_52/52_3.JPEG',
                              '/depth_52/52_4.JPEG', '/depth_52/52_5.JPEG', '/depth_52/52_6.JPEG',
                              '/depth_52/52_7.JPEG'],
                       crop_coordinates=[[(246, 303), (478, 498)], [(235, 350), (449, 517)], [(315, 371), (535, 551)],
                                         [(293, 338), (531, 512)], [(311, 327), (548, 493)], [(301, 305), (545, 480)],
                                         [(337, 317), (590, 515)], [(223, 325), (474, 487)]],
                       centroids_to_ignore={},
                       imaging_depth_meters=0.052,
                       segmentation_verified=True)

# ----------------------------------------------------------------------------------------------------------------------
# The following keys are used in this script and their expected values are defined.
ENTER_KEY: int = int(13)
ENTER_STR: str = "'ENTER'"
ESC_KEY: int = int(27)
ESC_STR: str = "'ESC'"
R_KEY: int = int(114)
R_STR: str = "'R'"

# If the key values need to be verified, set to True
if verify_key_values:
    for key_value, key_string in zip([ENTER_KEY, ESC_KEY, R_KEY],
                                     [ENTER_STR, ESC_STR, R_STR]):
        # Show a blank image and wait for a key to be pressed
        imshow("Press " + key_string + " Key", ones((100, 100), dtype=uint8) * uint8(255))
        key_pressed = waitKey(-1)

        # Destroy the window
        destroyAllWindows()

        # Notify the user of the result
        if key_pressed != key_value:
            raise ValueError(key_string + " key value was expected to be " + str(key_value) + " but " +
                             str(key_pressed) + " was returned.")
        else:
            print(key_string + " key verified.")

# ----------------------------------------------------------------------------------------------------------------------
else:
    for data_set in [depth45_data, depth50_data, depth52_data]:

        print('-' * 27)
        print(('-' * 7) + ' ' + data_set.name + ' ' + ('-' * 7))
        print('-' * 27)

        # Define a place to store the pixel distances calculated
        vertical_distances = []
        horizontal_distances = []

        # Create a list of combined coordinates for easier use
        combined_coordinates = []

        # Create a list of the combined centroids to exclude to easier use
        combined_centroids = ''

        for img_path, crop_coordinate_pair in zip(data_set.paths, data_set.crop_coordinates):

            # Load the image
            img = imread(base_image_path + img_path)

            # Select the points around the dots
            if crop_coordinate_pair is None:
                first_pt = display_image_with_callback(img, 'Select the upper left corner', [0, 0])
                second_pt = display_image_with_callback(img, 'Select the upper right corner', [0, 0])
                combined_coordinates.append([tuple(first_pt), tuple(second_pt)])
            else:
                first_pt = crop_coordinate_pair[0]
                second_pt = crop_coordinate_pair[1]

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

            # Pop out any centroids that are marked as invalid
            if img_path in data_set.centroids_to_ignore.keys() and len(data_set.centroids_to_ignore[img_path]) > 0:
                for centroid in data_set.centroids_to_ignore[img_path]:
                    contour_centroids.pop(contour_centroids.index(list(centroid)))

            # Display the cropped image and the generated mask
            if not data_set.segmentation_verified:
                # Recolor the images to display them
                colored_cropped_image = cvtColor(cropped_image, COLOR_GRAY2BGR)
                colored_cropped_image_mask = cvtColor(cropped_image_mask * uint8(255), COLOR_GRAY2BGR)

                # Set the loop break variable
                break_out_of_loop = False

                # Create a list to store the centroids to remove
                centroids_to_remove = []

                while not break_out_of_loop:
                    # Add the centroids to each image
                    annotated_cropped_image = deepcopy(colored_cropped_image)
                    for centroid in contour_centroids:
                        circle(annotated_cropped_image, centroid, 4, (255, 0, 0), -1)
                    annotated_cropped_image_mask = deepcopy(colored_cropped_image_mask)
                    for centroid in contour_centroids:
                        circle(annotated_cropped_image_mask, centroid, 4, (0, 0, 255), -1)

                    # Display the images and wait for user to approve
                    imshow("Cropped Image and Corresponding Mask.",
                           append(arr=annotated_cropped_image, values=annotated_cropped_image_mask, axis=1))
                    key_pressed = waitKey(-1)

                    # If the user wants to remove points
                    if key_pressed == R_KEY:

                        # Remove the current window
                        destroyWindow("Cropped Image and Corresponding Mask.")

                        # Prompt the user to select a centroid to remove
                        selected_point = display_image_with_callback(annotated_cropped_image,
                                                                     "Select a centroid to remove.",
                                                                     [None, None])

                        # Find the closest centroid
                        centroids_to_remove.append(find_closest_point(tuple(selected_point), contour_centroids,
                                                                      use_norm=True))

                    # If the user wants to approve the segmentation
                    elif key_pressed == ENTER_KEY or key_pressed == ESC_KEY:
                        destroyAllWindows()
                        break_out_of_loop = True

                # Define the centroids to remove for the user to input at the top of the script
                if len(centroids_to_remove) > 0:
                    combined_centroids = combined_centroids + "'" + img_path + "': " + str(centroids_to_remove) + ', '

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
                            horizontal_distances.append(norm(array(rows[y][x]) - array(rows[y][x - 1])))
                        if y > 0:
                            if rows[y - 1][x] is not None:
                                vertical_distances.append(norm(array(rows[y][x]) - array(rows[y - 1][x])))

        # Print the combined coordinates if they were filled in
        if len(combined_coordinates) > 0:
            print('All Coordinates: ' + str(combined_coordinates))
            print('-' * 27)

        if len(combined_centroids) > 0:
            print('All Centroids  : ' + str(combined_centroids[:-2]))
            print('-' * 27)

        HORIZONTAL = 'Horizontal - '
        VERTICAL = 'Vertical --- '
        PX = ' (px): '
        print(('-' * 9) + ' Summary ' + ('-' * 9))
        print('-' * 27)
        print(HORIZONTAL + 'Min' + PX + str(round(min(horizontal_distances), 2)))
        print(HORIZONTAL + 'Max' + PX + str(round(max(horizontal_distances), 2)))
        print('-' * 27)
        print(HORIZONTAL + 'Mean   ' + PX + str(round(mean(horizontal_distances), 4)))
        print(HORIZONTAL + 'Median ' + PX + str(round(median(horizontal_distances), 4)))
        print(HORIZONTAL + 'Std Dev' + PX + str(round(stdev(horizontal_distances), 4)))
        print('-' * 27)
        print(HORIZONTAL + 'Res w/ Mean (m/px): ' +
              str(round(HORIZONTAL_PHANTOM_DISTANCE / mean(horizontal_distances), 8)))
        print(HORIZONTAL + 'Res w/ Min  (m/px): ' +
              str(round(HORIZONTAL_PHANTOM_DISTANCE / min(horizontal_distances), 8)))
        print(HORIZONTAL + 'Res w/ Max  (m/px): ' +
              str(round(HORIZONTAL_PHANTOM_DISTANCE / max(horizontal_distances), 8)))
        print('-' * 27)
        print(VERTICAL + 'Min' + PX + str(round(min(vertical_distances), 2)))
        print(VERTICAL + 'Max' + PX + str(round(max(vertical_distances), 2)))
        print('-' * 27)
        print(VERTICAL + 'Mean   ' + PX + str(round(mean(vertical_distances), 4)))
        print(VERTICAL + 'Median ' + PX + str(round(median(vertical_distances), 4)))
        print(VERTICAL + 'Std Dev' + PX + str(round(stdev(vertical_distances), 4)))
        print('-' * 27)
        print(VERTICAL + 'Res w/ Mean (m): ' + str(round(VERTICAL_PHANTOM_DISTANCE / mean(vertical_distances), 8)))
        print(VERTICAL + 'Res w/ Min  (m): ' + str(round(VERTICAL_PHANTOM_DISTANCE / min(vertical_distances), 8)))
        print(VERTICAL + 'Res w/ Max  (m): ' + str(round(VERTICAL_PHANTOM_DISTANCE / max(vertical_distances), 8)))
        print('-' * 27)
        print('Calculated Resolution (m/px): ' + str(round(data_set.imaging_depth_meters / 400, 8)))
