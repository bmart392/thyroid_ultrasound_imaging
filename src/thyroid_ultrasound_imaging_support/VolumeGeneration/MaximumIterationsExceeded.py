"""
File containing the MaximumIterationsExceeded class definition.
"""


# Import standard python packages

# Import custom python packages


class MaximumIterationsExceeded(Exception):

    def __init__(self, message: str):

        # Save the message given
        self.message = message

        # Call the constructor of the inherited class
        super().__init__(self.message)


if __name__ == '__main__':
    # Create an instance of the object for testing
    test_object = MaximumIterationsExceeded('Testing')
