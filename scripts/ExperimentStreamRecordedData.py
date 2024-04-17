#!/usr/bin/env python3

"""
File containing code to stream recorded data as ROS Topic.
"""

# TODO - Dream - Convert this into standard node format
# TODO - Dream - Add logging through BasicNode class
# TODO - Dream - Add ability to play images in reverse order
# TODO - Dream - Add proper try-cath error checking everywhere and incorporate logging into it
# TODO - Dream - Replace node name with constant from NodeNameConstants
# TODO - Dream - Add proper status publishing

# Import ROS specific packages
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Import standard packages
from cv2 import cvtColor, COLOR_BGR2GRAY, imread
from sys import stdout
from os.path import isdir

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *

# Import custom python packages
from thyroid_ultrasound_imaging_support.RegisteredData.generate_ordered_list_of_directory_contents import \
    generate_ordered_list_of_directory_contents


class ExperimentStreamRecordedData(BasicNode):

    def __init__(self):

        # Call the super node
        super().__init__()

        # Define flag to control image streaming
        self.stream_images = False
        self.restart_image_stream = False
        self.playback_images_in_reverse = False
        self.path_to_images_changed = False

        # Define a variable to store the image streaming_frequency
        self.streaming_frequency = 20.

        # Define variable to store the path to the images
        self.path_to_images = None

        # Define global iterator variable
        self.ii = 0

        # Define list where images are stored
        self.created_objects = None

        # Create a ROS node
        init_node(CLARIUS_US_SPOOF)

        # Create a publisher for the images
        self.us_pub = Publisher(IMAGE_SOURCE, Image, queue_size=100)

        # Create image streaming services
        Service(CS_IMAGE_LOCATION, StringRequest, self.image_location_handler)
        Service(CS_IMAGE_STREAMING_CONTROL, BoolRequest, self.streaming_commands_handler)
        Service(CS_IMAGE_STREAMING_RESTART, BoolRequest, self.restart_streaming_command_handler)
        Service(CS_IMAGE_STREAMING_REVERSE_PLAYBACK_DIRECTION, BoolRequest, self.reverse_playback_direction_handler)
        Service(CS_IMAGE_STREAMING_SET_FREQUENCY, Float64Request, self.set_frequency_handler)

    def image_location_handler(self, req: StringRequestRequest):
        if isdir(req.value):
            self.path_to_images_changed = self.path_to_images != req.value
            if self.path_to_images_changed:
                self.path_to_images = req.value
            return StringRequestResponse(True, NO_ERROR)
        else:
            return StringRequestResponse(False, 'Directory is invalid.')

    def streaming_commands_handler(self, req: BoolRequestRequest):
        self.stream_images = req.value
        return BoolRequestResponse(True, NO_ERROR)

    def restart_streaming_command_handler(self, req: BoolRequestRequest):
        self.restart_image_stream = req.value
        return BoolRequestResponse(True, NO_ERROR)

    def reverse_playback_direction_handler(self, req: BoolRequestRequest):
        self.playback_images_in_reverse = req.value
        return BoolRequestResponse(True, NO_ERROR)

    def set_frequency_handler(self, req: Float64RequestRequest):
        self.streaming_frequency = req.value
        return Float64RequestResponse(True, NO_ERROR)

    def main(self):

        if self.stream_images and self.created_objects is not None and not self.restart_image_stream and \
                not self.path_to_images_changed:

            # If the end of the stream has not been reached and the stream does not need to be restarted
            if ((not self.playback_images_in_reverse and self.ii < len(self.created_objects)) or
                    (self.playback_images_in_reverse and self.ii >= 0)):
                # Recolor the image to grayscale
                bscan = cvtColor(self.created_objects[self.ii], COLOR_BGR2GRAY)

                # Generate an image message to publish
                resulting_image_message: Image = CvBridge().cv2_to_imgmsg(bscan, encoding="passthrough")

                # Register when the image was taken
                resulting_image_message.header.stamp = Time.now()

                # Publish the image
                self.us_pub.publish(resulting_image_message)

                # Print how many images have been sent
                stdout.write(f'Image {self.ii + 1} of {len(self.created_objects)} sent.\r')
                stdout.flush()

                # Increment the iterator
                if self.playback_images_in_reverse:
                    change = -1
                else:
                    change = 1
                self.ii = self.ii + change

        elif self.restart_image_stream:

            if self.playback_images_in_reverse:
                self.ii = len(self.created_objects) - 1
            else:
                self.ii = 0

            self.restart_image_stream = False

        elif self.created_objects is None or self.path_to_images_changed:

            if self.path_to_images is not None:

                # Pull out the images from the given folder location
                try:
                    self.created_objects = [imread(path) for path in generate_ordered_list_of_directory_contents(
                        self.path_to_images, sort_indices=(0, 1))]
                except Exception:
                    try:
                        self.created_objects = [imread(path) for path in generate_ordered_list_of_directory_contents(
                            self.path_to_images, sort_indices=tuple([0]))]
                    except Exception:
                        raise Exception('Image files not named as expected.')
                # Reset the flag
                self.path_to_images_changed = False

                # Reset the image counter appropriately
                if self.playback_images_in_reverse:
                    self.ii = len(self.created_objects) - 1
                else:
                    self.ii = 0


if __name__ == '__main__':

    node = ExperimentStreamRecordedData()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # Define the publishing rate for these images
    pub_rate = Rate(node.streaming_frequency)  # Hz

    # Save the previous streaming frequency
    previous_streaming_frequency = node.streaming_frequency

    while not is_shutdown():

        # Run the main loop
        node.main()

        # If the streaming frequency has changed,
        if previous_streaming_frequency != node.streaming_frequency:

            # Update the publishing rate and save the old value
            pub_rate = Rate(node.streaming_frequency)
            previous_streaming_frequency = node.streaming_frequency

        # Sleep
        pub_rate.sleep()

    print("Node terminated.")
