from cv2 import imshow, waitKey, morphologyEx, MORPH_OPEN, MORPH_CLOSE, MORPH_DILATE, MORPH_ERODE
from numpy import newaxis, mean, std, where, uint8, ones, median
from time import perf_counter
from matplotlib.pyplot import subplots, draw, ion, show

from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_saved_image_data import load_folder_of_saved_image_data
from thyroid_ultrasound_imaging_support.Visualization.create_mask_overlay_array import create_mask_overlay_array, COLORIZED
from thyroid_ultrasound_imaging_support.Visualization.stitch_image_arrays import stitch_image_arrays


image_data_objects = load_folder_of_saved_image_data('/home/ben/thyroid_ultrasound_data/experimentation/'
                                             'VolumeGeneration/2024-04-10_15-24-00-724108_experiment/'
                                             'ImageData_2024-04-10_15-24-00-724108')
#
# ion()
#
# fig, axis = subplots(nrows=1, ncols=1)

for image_data_object in image_data_objects:
    image_data_object: ImageData
    imshow('foreground', create_mask_overlay_array(image_data_object.down_sampled_image,
                                                       image_data_object.image_mask,
                                                       overlay_method=COLORIZED,
                                                       overlay_color=(0, 25, 0)))
    imshow('mask', image_data_object.down_sampled_image * image_data_object.image_mask[:, :, newaxis])

    start_time = perf_counter()
    region_of_interest = image_data_object.down_sampled_image[:, :, 0] * image_data_object.image_mask
    non_zero_values = region_of_interest[region_of_interest > 0]
    mean_intensity = median(non_zero_values)
    std_dev_intensity = std(non_zero_values)
    lower_bound = mean_intensity - (2.0 * std_dev_intensity)
    upper_bound = mean_intensity + (1.0 * std_dev_intensity)

    modified_mask_a = (lower_bound < region_of_interest).astype('int')
    modified_mask_b = (region_of_interest < upper_bound).astype('int')
    modified_mask = modified_mask_a + modified_mask_b
    modified_mask = (modified_mask == 2).astype(uint8)
    # modified_mask = morphologyEx(modified_mask, MORPH_ERODE, ones((2, 2)), iterations=2)
    # modified_mask = morphologyEx(modified_mask, MORPH_CLOSE, ones((2, 2)), iterations=2)

    region_of_interest = image_data_object.down_sampled_image[:, :, 0] * modified_mask
    non_zero_values = region_of_interest[region_of_interest > 0]
    median_intensity = median(non_zero_values)
    std_dev_intensity = std(non_zero_values)
    lower_bound = (0.65 * median_intensity - (1.5 * std_dev_intensity))
    upper_bound = (0.65 * median_intensity + (1.5 * std_dev_intensity))

    modified_mask_c = (lower_bound < region_of_interest).astype('int')
    modified_mask_d = (region_of_interest < upper_bound).astype('int')
    modified_mask_nodule = modified_mask_c + modified_mask_d
    modified_mask_nodule = (modified_mask_nodule == 2).astype(uint8)

    print('Mask Generation: ' + str(round(1/(perf_counter() - start_time), 1)) + ' Hz')

    imshow('modified mask', image_data_object.down_sampled_image * modified_mask[:, :, newaxis])

    imshow('nodule', image_data_object.down_sampled_image * modified_mask_nodule[:, :, newaxis])
    # axis.cla()
    # axis.hist(non_zero_values.flatten(), bins=200)
    # draw()


    waitKey(60)
