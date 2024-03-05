from skimage.segmentation import (morphological_chan_vese,
                                  morphological_geodesic_active_contour,
                                  inverse_gaussian_gradient,
                                  checkerboard_level_set)
from skimage.data import coins
from matplotlib.pyplot import imread, subplots, show
from cv2 import cvtColor, COLOR_RGBA2GRAY, waitKey, convertScaleAbs, COLOR_GRAY2RGB
from cv2 import imshow as imshow2
from numpy import zeros, int8, uint8, newaxis

fig, axes = subplots(2, 2)
ax = axes.flatten()

test_img = imread(
    '/home/ben/thyroid_ultrasound_data/testing_and_validation/raw_images/2023-11-29_19-14/Slice_00120.png')
test_img = cvtColor(test_img, COLOR_RGBA2GRAY)
test_img = (test_img * 255).astype(uint8)
ax[0].imshow(test_img, cmap='gray')

initial_ls = zeros(test_img.shape, dtype=int8)
initial_ls[225:400, 190:400] = 1


gi_img = inverse_gaussian_gradient(test_img) * 255

ax[1].imshow(convertScaleAbs(cvtColor(test_img, COLOR_GRAY2RGB), alpha=1.5, beta=0))

ax[2].imshow(initial_ls * 255, cmap='gray')

ls = morphological_chan_vese(image=test_img,
                             num_iter=100,
                             init_level_set=initial_ls,
                             smoothing=4,
                             lambda1=2.5,
                             lambda2=1
                             )
ax[3].imshow(test_img, cmap='gray')
ax[3].contour(ls, [0.5], colors='r')
show()
