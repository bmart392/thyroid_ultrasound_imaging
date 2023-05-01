"""
Contains constants used in image filter objects.
"""

# Define constants to be used by any image filter implementing this interface
COLOR_BGR: int = 0
COLOR_RGB: int = 1
COLOR_GRAY: int = 2

NO_BLUR: int = 0
BLUR_GAUSSIAN: int = 1
BLUR_MEAN_FILTER: int = 2

THRESHOLD_BASIC: int = 0
THRESHOLD_ADAPTIVE: int = 1

MASK_ERODE: int = 0
MASK_DILATE: int = 1
MASK_OPEN: int = 2
MASK_CLOSE: int = 3
MASK_RECT: int = 4
