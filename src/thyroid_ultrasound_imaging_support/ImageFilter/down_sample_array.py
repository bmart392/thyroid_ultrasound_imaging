"""
Contains the function definition for the down-sample function
"""

from numpy import array, zeros, ceil, arange, product, ones


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
            return_array[int(ii/rate)][int(jj/rate)] = input_array[ii][jj]

    return return_array


def up_sample(down_sampled_array: array, destination_array: array, rate: int = 2):

    rate = int(ceil(rate))

    if not len(down_sampled_array.shape) == len(destination_array.shape):
        raise Exception("The shapes of the down-sampled and destination arrays do not match."
                        " Down-sampled array is " + str(len(down_sampled_array.shape)) +
                        "D and destination array is " + str(len(destination_array.shape)) + "D.")
    if (down_sampled_array.shape[0] * rate) - 1 > destination_array.shape[0] or (down_sampled_array.shape[1] * rate) - 1 > destination_array.shape[1]:
        raise Exception("The down-sampled array has more points than the destination array allows at the current "
                        "sampling rate.\n" + "              Down-sampled array has shape " + str(down_sampled_array.shape) +
                        ", destination array has shape " + str(destination_array.shape) +
                        ", and the sampling rate is " + str(rate) + ".")

    for ii in range(down_sampled_array.shape[0]):
        for jj in range(down_sampled_array.shape[1]):
            destination_array[int(ii * rate)][int(jj * rate)] = down_sampled_array[ii][jj]

    return destination_array


def up_sample_to_zero_array(down_sampled_array: array, destination_array_shape: tuple = None, rate: int = 2):

    rate = int(ceil(rate))

    if destination_array_shape is None:
        zeros_array_shape = [int((down_sampled_array.shape[0] * rate) - 1), int((down_sampled_array.shape[1] * rate) - 1)]
        if len(down_sampled_array.shape) == 3:
            zeros_array_shape.append(int(down_sampled_array.shape[2]))
    else:
        zeros_array_shape = [int(destination_array_shape[0]), int(destination_array_shape[1])]
        if len(destination_array_shape) == 3:
            zeros_array_shape.append(int(destination_array_shape[2]))

    return up_sample(down_sampled_array, zeros(zeros_array_shape), rate=rate)





if __name__ == '__main__':

    array_shape = (5, 5)
    a = arange(product(array_shape)).reshape(array_shape)

    b = down_sample_array(a, rate=2)
    b_shape = b.shape
    b = b.reshape(product(b_shape))

    for pp in range(product(b_shape)):
        b[pp] = b[pp] * 10

    b = b.reshape(b_shape)

    c = up_sample(b, a, rate=2)

    d = up_sample_to_zero_array(b, array_shape, rate=2)

    print("Done")