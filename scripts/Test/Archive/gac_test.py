from skimage.segmentation import (morphological_chan_vese,
                                  morphological_geodesic_active_contour,
                                  inverse_gaussian_gradient,
                                  checkerboard_level_set)
from skimage.data import coins
from matplotlib.pyplot import imread, subplots, show
from cv2 import cvtColor, COLOR_RGBA2GRAY, waitKey, convertScaleAbs, COLOR_GRAY2RGB, GC_FGD, GC_PR_FGD
from cv2 import imshow as imshow2
from numpy import zeros, int8, uint8, newaxis, where, array, meshgrid, ones

from thyroid_ultrasound_imaging_support.Boundaries.create_convex_triangles_from_points import \
    create_convex_triangles_from_points
from thyroid_ultrasound_imaging_support.Boundaries.create_previous_image_mask_array_from_triangles import \
    create_previous_image_mask_array_from_triangles
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.UserInput.user_input_polygon_points import user_input_polygon_points

fig, axes = subplots(2, 2)
ax = axes.flatten()

test_img = imread(
    '/home/ben/thyroid_ultrasound_data/testing_and_validation/raw_images/2023-11-29_19-14/Slice_00120.png')
test_img = cvtColor(test_img, COLOR_RGBA2GRAY)
test_img = (test_img * 255).astype(uint8)
ax[0].imshow(test_img, cmap='gray')

test_img_data = ImageData(test_img, image_capture_time=123)
test_img_data.colorized_image = test_img

list_of_points_for_foreground_polygon = user_input_polygon_points(
                        test_img_data,
                        "area of interest",
                        display_result=True,
                        list_of_points=None)

list_of_foreground_triangles = create_convex_triangles_from_points(
                        list_of_points_for_foreground_polygon)

initialization_mask = create_previous_image_mask_array_from_triangles(
                        [],  # list_of_background_triangles,
                        list_of_foreground_triangles,
                        test_img_data.colorized_image.shape[:2])

initialization_mask_FGD = where(initialization_mask == GC_FGD, GC_PR_FGD, 0)
initialization_mask_PR_FGD = where(initialization_mask == GC_PR_FGD, GC_PR_FGD, 0)
initialization_mask = int8((initialization_mask_FGD + initialization_mask_PR_FGD) / GC_PR_FGD)


initial_ls = zeros(test_img.shape, dtype=int8)
initial_ls[225:400, 190:400] = 1


gi_img = inverse_gaussian_gradient(test_img) * 255

ax[1].imshow(convertScaleAbs(cvtColor(test_img, COLOR_GRAY2RGB), alpha=1.5, beta=0))
ax[2].imshow(initialization_mask * 255, cmap='gray')

ls = morphological_chan_vese(image=test_img,
                             num_iter=100,
                             init_level_set=initialization_mask,
                             smoothing=4,
                             lambda1=5,
                             lambda2=1
                             )
ax[3].imshow(test_img, cmap='gray')
ax[3].contour(ls, [0.5], colors='r')
show()
