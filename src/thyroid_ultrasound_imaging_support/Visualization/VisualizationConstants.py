"""
Contains constants used to define visualizations.
"""
# Define a constant for each type of image stream
IMG_SINGLE: int = 0
IMG_CONTINUOUS: int = 1

# Define a constant for each visualization option
SHOW_ORIGINAL: int = 0
SHOW_CROPPED: int = 1
SHOW_RECOLOR: int = 2
SHOW_BLUR: int = 3
SHOW_MASK: int = 5
SHOW_POST_PROCESSED_MASK: int = 6
SHOW_SURE_FOREGROUND: int = 7
SHOW_SURE_BACKGROUND: int = 8
SHOW_PROBABLE_FOREGROUND: int = 9
SHOW_INITIALIZED_MASK: int = 10
SHOW_CENTROIDS_ONLY: int = 11
SHOW_CENTROIDS_CROSS_ONLY: int = 12
SHOW_MASK_OVERLAY: int = 13
SHOW_CENTROIDS_CROSS_OVERLAY: int = 14
SHOW_MASK_CENTROIDS_CROSS_OVERLAY: int = 15
SHOW_FOREGROUND: int = 16
SHOW_SKIN_APPROXIMATION: int = int(17)
SHOW_GRABCUT_USER_INITIALIZATION_0: int = 100
SHOW_GRABCUT_USER_INITIALIZATION_1: int = 101
SHOW_GRABCUT_USER_INITIALIZATION_3: int = 103

INITIALIZED_MASK_MULTIPLIER: int = 50
FULL_VALUE_RGB_MULTIPLIER: int = 255

# Define constants used to visualize the skin approximation
SKIN_APPROXIMATION_MODE: int = int(0)
BEST_FIT_LEFT_LINE_A: int = int(1)
BEST_FIT_LEFT_LINE_B: int = int(2)
BEST_FIT_RIGHT_LINE_A: int = int(3)
BEST_FIT_RIGHT_LINE_B: int = int(4)
BEST_FIT_SHARED_LINE_A: int = int(5)
BEST_FIT_SHARED_LINE_B: int = int(6)

