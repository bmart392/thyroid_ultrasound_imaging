from enum import Enum


class ColorOptions(Enum):
    BGR, RGB, GRAY = range(3)


class BlurTypes(Enum):
    NO_BLUR, GAUSSIAN, MEAN_FILTER = range(3)


class ContrastOptions(Enum):
    ORIGINAL, BALANCED = range(2)


class ThresholdOptions(Enum):
    BASIC, ADAPTIVE = range(2)


class MaskManipulationOptions(Enum):
    RECT, CLOSE, OPEN, ERODE = range(4)
