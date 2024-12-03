"""
File containing the Point class definition.
"""


# Import standard python packages

# Import custom python packages


class Point:
    def __init__(self,
                 index: int = None,
                 value: tuple = None):
        self.index: int = index
        self.value: tuple = value


if __name__ == '__main__':
    # Create an instance of the object for testing
    test_object = Point()
