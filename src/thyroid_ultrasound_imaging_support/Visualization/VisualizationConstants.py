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

# Define a title for each visualization option
TITLE_ORIGINAL: str = "Original Image"
TITLE_CROPPED: str = "Cropped Image"
TITLE_RECOLOR: str = "Recolored Image"
TITLE_PRE_PROCESSED: str = "Pre-processed Image"
TITLE_RESULT_MASK: str = "Image Mask\nfrom Segmentation"
TITLE_POST_PROCESSED_MASK: str = "Post-processed Image Mask\nfrom Segmentation"
TITLE_SURE_FOREGROUND: str = "Sure Foreground of Image"
TITLE_SURE_BACKGROUND: str = "Sure Background of Image"
TITLE_PROBABLE_FOREGROUND: str = "Probable Foreground of Image"
TITLE_INITIALIZED_MASK: str = "Mask for Initializing\nSegmentation"
TITLE_CENTROIDS_ONLY: str = "Image Centroids"
TITLE_CENTROIDS_CROSS_ONLY: str = "Image Centroids with Goal Line."
TITLE_MASK_OVERLAY: str = "Mask Overlaid\non Original Image."
TITLE_CENTROIDS_CROSS_OVERLAY: str = "Centroids Overlaid\non Original Image."
TITLE_MASK_CENTROIDS_CROSS_OVERLAY: str = "Mask, Centroids, and Cross\nOverlaid on Original Image."
TITLE_FOREGROUND: str = "Foreground of the Image"
TITLE_BALANCE_LINE_APPROXIMATION: str = "Balance Line Approximation"
TITLE_GRABCUT_USER_INITIALIZATION_0: str = "Initialized Region of Image"

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

