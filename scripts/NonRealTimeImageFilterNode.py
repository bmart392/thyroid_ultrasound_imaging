#!/usr/bin/env python3

"""
File containing the NonRealTimeImageFiterNode class definition and ROS running code.
"""

# TODO - Dream - Add logging through BasicNode class
# TODO - Dream - Add proper try-cath error checking everywhere and incorporate logging into it

# Import standard python packages
from os.path import isdir
from time import time
from cv2 import resize, INTER_CUBIC

# Import standard ROS packages

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilterGrabCut import ImageFilterGrabCut
from thyroid_ultrasound_imaging_support.RegisteredData.MessageCompatibleObject import TO_MESSAGE, SAVE_OBJECT
from thyroid_ultrasound_imaging_support.RegisteredData.RegisteredData import RegisteredData
from thyroid_ultrasound_imaging_support.RegisteredData.load_folder_of_saved_registered_data import \
    load_folder_of_saved_registered_data
from thyroid_ultrasound_imaging_support.Validation.build_previous_image_mask_from_ground_truth import \
    build_previous_image_mask_from_ground_truth
from thyroid_ultrasound_imaging_support.Visualization.display_process_timer import display_process_timer

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_messages.msg import RegisteredDataMsg, NonRealTimeImageFilterStatus


class NonRealTimeImageFilterNode(BasicNode):

    def __init__(self):

        # Call the constructor of the super class
        super(NonRealTimeImageFilterNode, self).__init__()

        # Define a variable to store the location from which to pull data
        self.registered_data_load_location: str = ""

        # Define a variable to store the data that will be segmented
        self.registered_data = []

        # Define a variable to store if the node has been commanded to generate a new volume
        self.commanded_to_generate_volume = False

        # Define the image filter to use to filter the images
        self.image_filter = ImageFilterGrabCut(down_sampling_rate=1.0,
                                               segmentation_iteration_count=8,
                                               sure_background_creation_iterations=8,
                                               sure_foreground_creation_iterations=8
                                               )

        # Create the node
        init_node(NON_REAL_TIME_IMAGE_FILTER)

        # Create a publisher to publish the registered data in non-real time
        self.registered_data_publisher = Publisher(REGISTERED_DATA_NON_REAL_TIME, RegisteredDataMsg, queue_size=1)

        # Create a publisher to publish the status of the image filter
        self.image_filter_progress_publisher = Publisher(REGISTERED_DATA_NON_REAL_TIME_SEGMENTATION_PROGRESS,
                                                         NonRealTimeImageFilterStatus, queue_size=1)

        # Define a subscriber to listen for where to look for the registered data
        Subscriber(REGISTERED_DATA_LOAD_LOCATION, String, self.registered_data_load_location_callback)

        # Define a subscriber to listen for the command to generate the volume
        Subscriber(GENERATE_VOLUME, Bool, self.generate_volume_command_callback)

    def registered_data_load_location_callback(self, msg: String):
        """
        Updates the internal registered_data_load_location with the path included in the message
        as long as it is a valid path to a directory.
        """
        if isdir(msg.data):
            self.registered_data_load_location = msg.data

    def publish_segmentation_status(self, number_of_images_segmented: int, number_of_images_to_segment: int):
        """
        Publishes the progress of the segmentation as a NonRealTimeImageFilterStatus message.
        """
        new_msg = NonRealTimeImageFilterStatus(
            is_all_data_filtered=number_of_images_segmented == number_of_images_to_segment,
            num_images_filtered=number_of_images_segmented,
            total_images_to_filter=number_of_images_to_segment)
        new_msg.header.stamp = Time.now()
        self.image_filter_progress_publisher.publish(new_msg)

    def generate_volume_command_callback(self, msg: Bool):
        """
        Updates the internal commanded_to_generate_volume flag with the value from the message.
        """
        self.commanded_to_generate_volume = msg.data

    def main_loop(self):
        """
        Segments the saved data when commanded to generate a volume and a valid path has been given.
        """
        if self.commanded_to_generate_volume:
            # Ensure that the specified directory exists
            if isdir(self.registered_data_load_location):

                # Load the registered data
                self.registered_data: list = load_folder_of_saved_registered_data(self.registered_data_load_location)

                # Populate the filter with the appropriate information from the first registered image data object
                first_image_data_object: ImageData = self.registered_data[0].image_data
                self.image_filter.image_crop_coordinates = first_image_data_object.image_crop_coordinates
                self.image_filter.image_crop_included = True
                self.image_filter.update_previous_image_mask(
                    build_previous_image_mask_from_ground_truth(
                        ground_truth_mask=resize(first_image_data_object.image_mask,
                                                 dsize=(first_image_data_object.cropped_image.shape[1],
                                                        first_image_data_object.cropped_image.shape[0]),
                                                 interpolation=INTER_CUBIC)))

                # Publish the status of the segmentation
                self.publish_segmentation_status(number_of_images_segmented=0,
                                                 number_of_images_to_segment=len(self.registered_data))

                # For each registered data point,
                for registered_data in self.registered_data:

                    # Stop the loop if the node was no longer commanded to generate a volume
                    if not self.commanded_to_generate_volume:
                        break

                    # Note when the process started
                    start_of_process_time = time()

                    # Define a new image data object using the old image data object as a reference
                    new_image_data_object = ImageData(image_data=registered_data.image_data.original_image,
                                                      image_color=registered_data.image_data.image_color,
                                                      image_capture_time=registered_data.image_data.image_capture_time,
                                                      image_title=NON_REAL_TIME_IMAGE_FILTER)

                    # Note the amount of time required to create an image data object
                    start_of_process_time = display_process_timer(start_of_process_time,
                                                                  "Image_data object creation time")

                    # Filter the image
                    self.image_filter.filter_image(new_image_data_object)

                    # Find the contours in the mask
                    new_image_data_object.generate_contours_in_image()

                    # Find the centroids of each contour
                    new_image_data_object.calculate_image_centroids()

                    # Note the amount of time required to analyze the image
                    display_process_timer(start_of_process_time, "Image analysis time")

                    # Update the registered data object with the newly filtered image
                    registered_data.image_data = new_image_data_object

                    registered_data: RegisteredData
                    registered_data.save_load(action=SAVE_OBJECT,
                                              path_to_file_location='/home/ben/thyroid_ultrasound_data/'
                                                                    'testing_and_validation'
                                                                    '/non_real_time_registered_data')

                    # Publish the new registered data
                    self.registered_data_publisher.publish(registered_data.convert_object_message(TO_MESSAGE))

                    # Publish the status of the segmentation
                    self.publish_segmentation_status(number_of_images_segmented=0,
                                                     number_of_images_to_segment=len(self.registered_data))

            else:
                raise Exception("No source location was specified for the registered data.")


if __name__ == '__main__':
    # create node object
    filter_node = NonRealTimeImageFilterNode()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # Run the main loop until the node is shutdown
    while not is_shutdown():
        filter_node.main_loop()

    print("Node Terminated.")