#!/usr/bin/env python3

"""
File containing code to stream recorded data as ROS Topic.
"""

# TODO - HIGH - Add toggle to stream data with current time stamp or old time stamp

# Import ROS specific packages

# Import standard packages
from cv2 import cvtColor, COLOR_BGR2GRAY, imread
from sys import stdout
from os.path import isdir

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_messages.msg import ImageWithTimeData

# Import custom python packages
from thyroid_ultrasound_support.Functions.generate_ordered_list_of_directory_contents import \
    generate_ordered_list_of_directory_contents
from thyroid_ultrasound_support.MessageConversion.convert_image_array_to_image_with_time_data_message import \
    convert_image_array_to_image_with_time_data_message


class ExperimentStreamRecordedData(BasicNode):

    def __init__(self):

        # Call the super node
        super().__init__()

        # Define flag to control image streaming
        self.stream_images = False
        self.restart_image_stream = False
        self.playback_images_in_reverse = False
        self.path_to_images_changed = False
        self.apply_new_timestamps = True

        # Define a variable to store the image streaming_frequency
        self.streaming_frequency = 20.

        # Define variable to store the path to the images
        self.path_to_images = None

        # Define global iterator variable
        self.ii = 0

        # Define a variable to control single image iterations
        self.single_image_iteration_count = 0
        self.single_image_iterations = 0

        # Define list where images are stored
        self.created_objects = None
        self.created_objects_sec = None
        self.created_objects_n_sec = None

        # Create a ROS node
        init_node(CLARIUS_US_SPOOF)

        # Create a publisher for the images
        self.us_pub = Publisher(IMAGE_SOURCE, ImageWithTimeData, queue_size=100)

        # Create image streaming services
        Service(CS_IMAGE_LOCATION, StringRequest, self.image_location_handler)
        Service(CS_IMAGE_STREAMING_CONTROL, BoolRequest, self.streaming_commands_handler)
        Service(CS_IMAGE_STREAMING_RESTART, BoolRequest, self.restart_streaming_command_handler)
        Service(CS_IMAGE_STREAMING_REVERSE_PLAYBACK_DIRECTION, BoolRequest, self.reverse_playback_direction_handler)
        Service(CS_IMAGE_STREAMING_SET_FREQUENCY, Float64Request, self.set_frequency_handler)
        Service(CS_APPLY_NEW_TIMESTAMP, BoolRequest, self.apply_new_timestamps_command_handler)

    def image_location_handler(self, req: StringRequestRequest):
        """Updates the directory from which to stream images."""

        # If the directory is valid,
        if isdir(req.value):

            # Note if the directory path has changed
            self.path_to_images_changed = self.path_to_images != req.value

            # If it has changed, save the new path
            if self.path_to_images_changed:
                self.path_to_images = req.value

            self.log_single_message('New image location selected')

            # Send the response
            return StringRequestResponse(True, NO_ERROR)

        # Otherwise, send a response noting that the directory is not valid
        else:
            self.log_single_message('New image location was invalid')
            return StringRequestResponse(False, 'Directory is invalid.')

    def streaming_commands_handler(self, req: BoolRequestRequest):
        """Updates the local variable based on the value included in the request."""
        self.stream_images = req.value
        if req.value:
            self.log_single_message('Streaming requested to start')
        else:
            self.log_single_message('Streaming request to stop')
        return BoolRequestResponse(True, NO_ERROR)

    def restart_streaming_command_handler(self, req: BoolRequestRequest):
        """Updates the local variable based on the value included in the request."""
        self.restart_image_stream = req.value
        return BoolRequestResponse(True, NO_ERROR)

    def reverse_playback_direction_handler(self, req: BoolRequestRequest):
        """Updates the local variable based on the values included in the request."""
        self.playback_images_in_reverse = req.value
        if req.value:
            self.log_single_message('Image streaming order reversed')
        else:
            self.log_single_message('Image streaming order normal')
        return BoolRequestResponse(True, NO_ERROR)

    def set_frequency_handler(self, req: Float64RequestRequest):
        """Updates the local variable based on the value included in the request."""
        self.streaming_frequency = req.value
        return Float64RequestResponse(True, NO_ERROR)

    def apply_new_timestamps_command_handler(self, req: BoolRequestRequest):
        """Updates the local variable based on the value included in the request."""
        self.apply_new_timestamps = req.value
        return BoolRequestResponse(True, NO_ERROR)

    def main(self):
        # Define the publishing rate for these images
        pub_rate = Rate(self.streaming_frequency)  # Hz

        # Save the previous streaming frequency
        previous_streaming_frequency = self.streaming_frequency

        while not is_shutdown():

            if self.stream_images and self.created_objects is not None and not self.restart_image_stream and \
                    not self.path_to_images_changed:

                # If the end of the stream has not been reached and the stream does not need to be restarted
                if ((not self.playback_images_in_reverse and self.ii < len(self.created_objects)) or
                        (self.playback_images_in_reverse and self.ii >= 0)):

                    try:
                        # Recolor the image to grayscale
                        bscan = cvtColor(self.created_objects[self.ii], COLOR_BGR2GRAY)

                        # Pull out the appropriate saved timestamp
                        try:
                            created_object_timestamp_sec = self.created_objects_sec[self.ii]
                            created_object_timestamp_nsec = self.created_objects_n_sec[self.ii]
                        except TypeError or IndexError:
                            created_object_timestamp_sec = None
                            created_object_timestamp_nsec = None

                        # Create the image message to publish
                        resulting_image_message = \
                            convert_image_array_to_image_with_time_data_message(
                                image_array=bscan,
                                apply_new_timestamp=self.apply_new_timestamps,
                                alternative_timestamp_sec=created_object_timestamp_sec,
                                alternative_timestamp_nsec=created_object_timestamp_nsec)

                        # Publish the image
                        self.us_pub.publish(resulting_image_message)

                        # Print how many images have been sent
                        stdout.write(f'Image {self.ii + 1} of {len(self.created_objects)} sent.\r')
                        stdout.flush()

                        # Increment the iterator
                        if self.single_image_iterations < self.single_image_iteration_count:
                            self.single_image_iterations = self.single_image_iterations + 1
                        else:
                            self.single_image_iterations = 0

                            if self.playback_images_in_reverse:
                                change = -1
                            else:
                                change = 1
                            self.ii = self.ii + change

                        self.publish_node_status(STREAMING_ACTIVE)

                    except TypeError:
                        self.log_single_message('Could not convert image')
                        self.created_objects = None
                        self.created_objects_sec = None
                        self.created_objects_n_sec = None
                        self.path_to_images = None
                        self.log_single_message('Images and filepaths cleared')

                else:
                    self.publish_node_status(ALL_IMAGES_STREAMED)

            elif self.restart_image_stream:

                if self.playback_images_in_reverse:
                    self.ii = len(self.created_objects) - 1
                else:
                    self.ii = 0

                self.restart_image_stream = False

                self.log_single_message('Image series restarted')

            elif (self.created_objects is None or self.path_to_images_changed) and self.path_to_images is not None:

                self.log_single_message('Reading in new images at ' + self.path_to_images)

                try:  # to pull out the images from the given folder location using the standard sort indices
                    self.created_objects_sec, self.created_objects_n_sec, \
                    ordered_file_paths = generate_ordered_list_of_directory_contents(
                        self.path_to_images, sort_indices=(1, 2), return_index_values=True)

                except IndexError:  # If the images cannot be parsed with the standard sort indices,
                    try:  # attempt to use the alternate sort indices
                        self.created_objects_sec, self.created_objects_n_sec, \
                        ordered_file_paths = generate_ordered_list_of_directory_contents(
                            self.path_to_images, sort_indices=(0, 1), return_index_values=True)

                    except ValueError:  # If the images still cannot be parsed,

                        # Allow sorting by strings and do not collect the timestamp of each image from the file name.
                        ordered_file_paths = list(
                            generate_ordered_list_of_directory_contents(self.path_to_images,
                                                                        sort_indices=(0, 1),
                                                                        sort_by_string_allowed=True))[
                            0]

                        # Reset the data stored here
                        self.created_objects_sec = None
                        self.created_objects_n_sec = None

                # Create images from each file path
                self.created_objects = [imread(path) for path in ordered_file_paths]

                # Reset the flag
                self.path_to_images_changed = False

                # Reset the image counter appropriately
                if self.playback_images_in_reverse:
                    self.ii = len(self.created_objects) - 1
                else:
                    self.ii = 0

                self.log_single_message('New images read in')

            else:

                self.publish_node_status(STREAMING_INACTIVE)

            # If the streaming frequency has changed,
            if previous_streaming_frequency != self.streaming_frequency:
                # Update the publishing rate and save the old value
                pub_rate = Rate(self.streaming_frequency)
                previous_streaming_frequency = self.streaming_frequency

                self.log_single_message('New streaming frequency is ' + str(self.streaming_frequency) + ' Hz')

            # Sleep
            pub_rate.sleep()


if __name__ == '__main__':
    node = ExperimentStreamRecordedData()

    node.log_single_message('Node initialized')

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # Run the main loop
    node.main()

    node.log_single_message('Node terminated')

    print("Node terminated.")
