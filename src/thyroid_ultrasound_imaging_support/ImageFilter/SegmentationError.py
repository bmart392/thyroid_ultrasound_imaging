"""
File containing the code for the SegmentationError class.
"""

# Define constants to tell which step of the segmentation failed
CROP_FAILURE: str = "Failed to crop the image"
COLORIZE_FAILURE: str = "Failed to colorize the image"
DOWN_SAMPLE_FAILURE: str = "Failed to down-sample the image"
PRE_PROCESS_FAILURE: str = "Failed to pre-process the image"
CREATE_MASK_FAILURE: str = "Failed to create the image mask"
POST_PROCESS_FAILURE: str = "Failed to post-process the mask"
CREATE_SURE_FOREGROUND_FAILURE: str = "Failed to create the sure-foreground mask"
CREATE_SURE_BACKGROUND_FAILURE: str = "Failed to create the sure-background mask"
CREATE_PROBABLE_FOREGROUND_FAILURE: str = "Failed to create the probable-foreground mask"
CREATE_PREVIOUS_IMAGE_MASK_FAILURE: str = "Failed to create the previous-image mask"
FILTER_IMAGE_FAILURE: str = "Failed to filter the image"


class SegmentationError(Exception):
    """
    This defines a common exception that can be used by any image segmentation system to note when the
    segmentation has failed.
    """

    def __init__(self, message: str, previous_history=None):
        """
        Creates a SegmentationError object.

        Parameters
        ----------
        message :
            The message to be displayed with the error.
        previous_history :
            A single string or a list of strings that will be passed along with the current message
        """

        # Call the constructor of the Exception class
        super().__init__(message)

        # Create the pass-through message field
        self.history = []

        # Define a default value for the passthrough-messages parameter
        if previous_history is not None:

            # Add the passed-through messages into the attribute based on the data type
            if type(previous_history) == str:
                self.history.append(previous_history)
            elif type(previous_history) == list:
                self.history.extend(previous_history)
            else:
                raise TypeError("The type given as a pass-though message must be str or list. A " +
                                str(type(previous_history)) + " object was given instead.")

        # Add the current message to the end
        self.history.append(message)

    def convert_history_to_string(self) -> str:
        """
        Convert the error history to a string with the history sorted in chronological order of newest to oldest.
        """

        # Sort the error history newest to oldest
        self.history.reverse()

        # Return the error history as a sting
        error_string = "Error history newest to oldest: " + str(self.history)

        # Sort the history back to its normal order
        self.history.reverse()

        # Return the actual error string
        return error_string


if __name__ == '__main__':
    try:
        try:
            raise SegmentationError("Oh?", "I think we have a problem.")
        except SegmentationError as test_error:
            raise SegmentationError("Yeah, it's getting weird.", test_error.history)
    except SegmentationError as test_error:
        print(test_error.convert_history_to_string())
