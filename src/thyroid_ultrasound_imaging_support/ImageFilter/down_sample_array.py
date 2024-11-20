"""
Contains the function definition for the down-sample function
"""

from numpy import array, zeros, ceil, arange, product, uint8
from matplotlib.pyplot import matshow, imread, subplots, pause
from cv2 import resize, INTER_AREA, INTER_CUBIC, INTER_LINEAR
from thyroid_ultrasound_imaging_support.Visualization.display_process_timer import *


def down_sample_array(input_array: array, rate: int = 2):
    rate = int(ceil(rate))

    len_of_shape_of_input_array = len(input_array.shape)
    if not len_of_shape_of_input_array == 2:
        if not len_of_shape_of_input_array == 3:
            raise Exception("Incorrect shape of input array given. Expects 2D or 3D, was given " +
                            str(len_of_shape_of_input_array) + "D.")

    new_array_shape = [int(ceil(input_array.shape[0] / rate)), int(ceil(input_array.shape[1] / rate))]

    if len_of_shape_of_input_array == 3:
        new_array_shape.append(int(input_array.shape[2]))

    return_array = zeros(new_array_shape)

    for ii in range(0, input_array.shape[0], rate):
        for jj in range(0, input_array.shape[1], rate):
            return_array[int(ii / rate)][int(jj / rate)] = input_array[ii][jj]

    return return_array


def up_sample(down_sampled_array: array, destination_array: array, rate: int = 2):
    rate = int(ceil(rate))

    if not len(down_sampled_array.shape) == len(destination_array.shape):
        raise Exception("The shapes of the down-sampled and destination arrays do not match."
                        " Down-sampled array is " + str(len(down_sampled_array.shape)) +
                        "D and destination array is " + str(len(destination_array.shape)) + "D.")
    if (down_sampled_array.shape[0] * rate) - 1 > destination_array.shape[0] or (
            down_sampled_array.shape[1] * rate) - 1 > destination_array.shape[1]:
        raise Exception("The down-sampled array has more points than the destination array allows at the current "
                        "sampling rate.\n" + "              Down-sampled array has shape " + str(
            down_sampled_array.shape) +
                        ", destination array has shape " + str(destination_array.shape) +
                        ", and the sampling rate is " + str(rate) + ".")

    for ii in range(down_sampled_array.shape[0]):
        for jj in range(down_sampled_array.shape[1]):
            destination_array[int(ii * rate)][int(jj * rate)] = down_sampled_array[ii][jj]

    return destination_array


def up_sample_mask_with_connection(down_sampled_array: array, destination_array_shape: tuple = None, rate: int = 2):
    rate = int(ceil(rate))

    if destination_array_shape is None:
        zeros_array_shape = [int((down_sampled_array.shape[0] * rate) - 1),
                             int((down_sampled_array.shape[1] * rate) - 1)]
        if len(down_sampled_array.shape) == 3:
            zeros_array_shape.append(int(down_sampled_array.shape[2]))
    else:
        zeros_array_shape = [int(destination_array_shape[0]), int(destination_array_shape[1])]
        if len(destination_array_shape) == 3:
            zeros_array_shape.append(int(destination_array_shape[2]))

    destination_array = zeros(zeros_array_shape)

    if not len(down_sampled_array.shape) == len(destination_array.shape):
        raise Exception("The shapes of the down-sampled and destination arrays do not match."
                        " Down-sampled array is " + str(len(down_sampled_array.shape)) +
                        "D and destination array is " + str(len(destination_array.shape)) + "D.")
    if (down_sampled_array.shape[0] * rate) - 1 > destination_array.shape[0] or (
            down_sampled_array.shape[1] * rate) - 1 > destination_array.shape[1]:
        raise Exception("The down-sampled array has more points than the destination array allows at the current "
                        "sampling rate.\n" + "              Down-sampled array has shape " + str(
            down_sampled_array.shape) +
                        ", destination array has shape " + str(destination_array.shape) +
                        ", and the sampling rate is " + str(rate) + ".")

    primary_directions_to_check = [(1, 0, arange(rate), zeros(rate)),
                                   (0, 1, zeros(rate), arange(rate))]  # (row, column)
    secondary_directions_to_check = [(1, -1, arange(rate), -arange(rate)),
                                     (1, 1, arange(rate), arange(rate))]  # (row, column)

    for ii in range(down_sampled_array.shape[0]):
        for jj in range(down_sampled_array.shape[1]):
            destination_array[int(ii * rate)][int(jj * rate)] = down_sampled_array[ii][jj]
            matshow(destination_array * 255, 0)
            if product(down_sampled_array[ii][jj]) > 0:
                primary_directions_succeeded = False
                for direction in primary_directions_to_check:
                    try:
                        if product(down_sampled_array[ii + direction[0]][jj + direction[1]]) > 0:
                            for delta_ii, delta_jj in zip(direction[2], direction[3]):
                                destination_array[int((ii * rate) + delta_ii)][int((jj * rate) + delta_jj)] = \
                                    down_sampled_array[ii][jj]
                                matshow(destination_array * 255, 0)
                            primary_directions_succeeded = True
                    except IndexError:
                        pass
                ii_to_check = ii
                jj_to_check = jj - 1
                value_to_check = down_sampled_array[ii][jj - 1]
                if not primary_directions_succeeded:
                    direction = (1, 1, arange(rate), arange(rate))
                    try:
                        if product(down_sampled_array[ii + direction[0]][jj + direction[1]]) > 0:
                            for delta_ii, delta_jj in zip(direction[2], direction[3]):
                                destination_array[int((ii * rate) + delta_ii)][int((jj * rate) + delta_jj)] = \
                                    down_sampled_array[ii][jj]
                                matshow(destination_array * 255, 0)
                    except IndexError:
                        pass
                if not primary_directions_succeeded and product(down_sampled_array[ii][[jj - 1]]) == 0:
                    direction = (1, -1, arange(rate), -arange(rate))
                    try:
                        if product(down_sampled_array[ii + direction[0]][jj + direction[1]]) > 0:
                            for delta_ii, delta_jj in zip(direction[2], direction[3]):
                                destination_array[int((ii * rate) + delta_ii)][int((jj * rate) + delta_jj)] = \
                                    down_sampled_array[ii][jj]
                                matshow(destination_array * 255, 0)
                    except IndexError:
                        pass

    return destination_array


def up_sample_to_zero_array(down_sampled_array: array, destination_array_shape: tuple = None, rate: int = 2):
    rate = int(ceil(rate))

    if destination_array_shape is None:
        zeros_array_shape = [int((down_sampled_array.shape[0] * rate) - 1),
                             int((down_sampled_array.shape[1] * rate) - 1)]
        if len(down_sampled_array.shape) == 3:
            zeros_array_shape.append(int(down_sampled_array.shape[2]))
    else:
        zeros_array_shape = [int(destination_array_shape[0]), int(destination_array_shape[1])]
        if len(destination_array_shape) == 3:
            zeros_array_shape.append(int(destination_array_shape[2]))

    return up_sample(down_sampled_array, zeros(zeros_array_shape), rate=rate)


if __name__ == '__main__':
    test_rate = 5
    """array_shape = (100, 100)
    a = arange(product(array_shape)).reshape(array_shape)

    b = down_sample_array(a, rate=rate)
    b_shape = b.shape
    b = b.reshape(product(b_shape))

    for pp in range(product(b_shape)):
        b[pp] = b[pp] * 10

    b = b.reshape(b_shape)

    c = up_sample(b, a, rate=rate)

    d = up_sample_to_zero_array(b, array_shape, rate=rate)"""

    """down_sampled_mask = array([[0, 0, 0, 1, 0],
                               [0, 1, 1, 0, 1],
                               [0, 1, 0, 1, 0],
                               [0, 1, 1, 1, 0],
                               [0, 0, 0, 0, 0]], dtype=uint8)

    full_mask = up_sample_mask_with_connection(down_sampled_mask, rate=rate)

    matshow(full_mask * 255, 0)
    show()"""

    fig, axes = subplots(2, 4, constrained_layout=True)
    test_image = imread("/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/"
                        "Test/Images/2023-11-29_19-14/Slice_00001.png")

    start_time = time()
    shrunk_image_inter_area = resize(test_image, (0, 0), fx=1 / test_rate, fy=1 / test_rate, interpolation=INTER_AREA)
    start_time, image_elapsed_time_inter_area = display_process_timer(start_time, "Shrink - INTER_AREA",
                                                                      return_time=True)
    shrunk_image_inter_cubic = resize(test_image, (0, 0), fx=1 / test_rate, fy=1 / test_rate, interpolation=INTER_CUBIC)
    start_time, image_elapsed_time_inter_cubic = display_process_timer(start_time, "Shrink - INTER_CUBIC",
                                                                       return_time=True)
    shrunk_image_inter_linear = resize(test_image, (0, 0), fx=1 / test_rate, fy=1 / test_rate,
                                       interpolation=INTER_LINEAR)
    start_time, image_elapsed_time_inter_linear = display_process_timer(start_time, "Shrink - INTER_LINEAR",
                                                                        return_time=True)

    small_mask = array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 1, 0, 0, 0, 0],
                        [0, 0, 0, 1, 1, 1, 0, 0, 0],
                        [0, 0, 1, 1, 1, 1, 1, 0, 0],
                        [0, 0, 0, 1, 1, 1, 0, 0, 0],
                        [0, 0, 0, 0, 1, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0]], dtype=uint8)

    test_rate = 10

    second_start_time = time()
    enlarged_mask_inter_area = resize(small_mask, (0, 0), fx=test_rate, fy=test_rate, interpolation=INTER_AREA)
    second_start_time, mask_elapsed_time_inter_area = display_process_timer(second_start_time, "Enlarge - INTER_AREA",
                                                                            return_time=True)
    enlarged_mask_inter_cubic = resize(small_mask, (0, 0), fx=test_rate, fy=test_rate, interpolation=INTER_CUBIC)
    second_start_time, mask_elapsed_time_inter_cubic = display_process_timer(second_start_time, "Enlarge - INTER_CUBIC",
                                                                             return_time=True)
    enlarged_mask_inter_linear = resize(small_mask, (0, 0), fx=test_rate, fy=test_rate, interpolation=INTER_LINEAR)
    second_start_time, mask_elapsed_time_inter_linear = display_process_timer(second_start_time,
                                                                              "Enlarge - INTER_LINEAR",
                                                                              return_time=True)

    im = axes[0][0].imshow(test_image)
    axes[0][0].set_title('Original Image')

    axes[0][1].imshow(shrunk_image_inter_area)
    axes[0][1].set_title('Image Shrunk - INTER_AREA\n' + 'Time to run (ms): ' + str(image_elapsed_time_inter_area))

    axes[0][2].imshow(shrunk_image_inter_cubic)
    axes[0][2].set_title('Image Shrunk - INTER_CUBIC\n' + 'Time to run (ms): ' + str(image_elapsed_time_inter_cubic))

    axes[0][3].imshow(shrunk_image_inter_linear)
    axes[0][3].set_title('Image Shrunk - INTER_LINEAR\n' + 'Time to run (ms): ' + str(image_elapsed_time_inter_linear))

    axes[1][0].imshow(small_mask * 255)
    axes[1][0].set_title('Original Mask')

    axes[1][1].imshow(enlarged_mask_inter_area)
    axes[1][1].set_title('Enlarged Mask - INTER_AREA\n' + 'Time to run (ms): ' + str(mask_elapsed_time_inter_area))

    axes[1][2].imshow(enlarged_mask_inter_cubic)
    axes[1][2].set_title('Enlarged Mask - INTER_CUBIC\n' + 'Time to run (ms): ' + str(mask_elapsed_time_inter_cubic))

    axes[1][3].imshow(enlarged_mask_inter_linear)
    axes[1][3].set_title('Enlarged Mask - INTER_LINEAR\n' + 'Time to run (ms): ' + str(mask_elapsed_time_inter_linear))

    # show()

    movie_images = [
        imread("/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/"
               "Test/Images/2023-11-29_19-14/Slice_00001.png"),
        imread("/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/"
               "Test/Images/2023-11-29_19-14/Slice_00026.png"),
        imread("/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/"
               "Test/Images/2023-11-29_19-14/Slice_00051.png"),
        imread("/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts/"
               "Test/Images/2023-11-29_19-14/Slice_00076.png")
    ]

    kk = 0

    while kk < 100:
        im.set_data(movie_images[kk % 4])
        fig.canvas.draw_idle()
        print(kk % 4)
        kk = kk + 1

        pause(1)

    print("Done")
