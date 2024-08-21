#!/usr/bin/env python3

"""
File containing ImagePositioningControllerNode class definition and ROS running code.
"""

# Import standard ROS packages
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import image_data_message

# Import custom packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_imaging_support.Controller.ImagePositioningControlConstants import IMAGE_CENTERING_OFFSET


class ImagePositioningControllerNode(BasicNode):

    def __init__(self):

        # Call the init of the super class
        super().__init__()

        # Define a variable to store the received images
        self.received_images = []

        # Define the maximum number of images to save
        self.max_images_to_store = 25

        # Define a variable to store the max allowed age of an image
        self.max_age_of_image_in_seconds = 0.5

        # Define a variable to store the imaging depth
        self.imaging_depth = 5.0

        # Define a variable to store the offset for the centering of the image
        self.image_centering_offset = 0

        # Define a flag to note when the ROI is shown in the image
        self.image_roi_shown = False

        # Create the node object
        init_node(IMAGE_POSITIONING_CONTROLLER)

        # Define a publisher to publish the image-based positioning error
        self.image_based_position_error_publisher = Publisher(
            RC_IMAGE_ERROR, TwistStamped, queue_size=1
        )

        # Define subscribers
        Subscriber(IMAGE_DEPTH, Float64, self.imaging_depth_callback)
        Subscriber(IMAGE_FILTERED, image_data_message, self.filtered_image_callback)
        Subscriber(RC_IMAGE_CENTERING_SIDE, Int8, self.image_centering_side_callback)
        Subscriber(IMAGE_ROI_SHOWN, Bool, self.image_roi_shown_callback)

        self.time_of_last_publishing = Time.now()
        self.log_single_message('Node initialized.')

    def imaging_depth_callback(self, msg: Float64):
        """Saves the newest imaging depth for later use."""
        # Save the newest imaging depth
        self.imaging_depth = msg.data

        self.log_single_message('New imaging depth of ' + str(self.imaging_depth) + ' cm saved')

    def filtered_image_callback(self, data: image_data_message):
        """
        Updates the list of received images and places the newest message at the end of the list.
        """
        # If the image is younger than the maximum allowed age,
        if abs((data.header.stamp - Time.now()).to_sec()) <= self.max_age_of_image_in_seconds:

            # Create new image data based on received image and add it to the list
            self.received_images.append(ImageData(image_data_msg=data))

            # Remove the oldest image if the list is now too long
            if len(self.received_images) > self.max_images_to_store:
                self.received_images.pop(0)

        else:
            self.log_single_message('Newly received image message was too old to be used')

    def image_centering_side_callback(self, msg: Int8):
        """
        Updates the stored image centering side.
        """
        self.image_centering_offset = msg.data * IMAGE_CENTERING_OFFSET

        self.log_single_message('Image centering offset set to ' + str(self.image_centering_offset))

    def image_roi_shown_callback(self, msg: Bool):
        """Updates the local variable noting if the ROI is shown in the image"""
        self.image_roi_shown = msg.data

    def publish_position_error(self):
        """Analyze the image to find the centroid position error."""
        # Create a local variable to set the status of the node
        current_status = None

        # Declare an empty value for the position error in case there is no image available
        position_error = None

        # Check that there is an image to use
        if len(self.received_images) > 0:

            # Pop out the newest data
            image_data = self.received_images.pop(-1)

            # Define a zero error value as default
            position_error = [0, 0, 0, 0, 0, 0]

            # If the ROI is in the image,
            if self.image_roi_shown:
                try:
                    # If more than one centroid exists, create a composited centroid location by
                    # averaging the location of all centroids in the image
                    if False and len(image_data.contour_centroids) > 1:
                        composite_centroid_location = sum(
                            [temp_centroid[0] for temp_centroid in image_data.contour_centroids]) / len(
                            image_data.contour_centroids)

                        current_status = CALCULATED_USING_COMPOSITE_CENTROID

                    # Otherwise, use the centroid location from the one valid centroid
                    else:
                        composite_centroid_location = image_data.contour_centroids[0][0]

                        current_status = CALCULATED_USING_SINGLE_CENTROID

                    # Calculate the X position error using the composite location
                    position_error[0] = composite_centroid_location - image_data.ds_image_size_x * (
                                self.image_centering_offset + 0.5)

                except IndexError:
                    current_status = ERROR_ACCESSING_CENTROIDS
                    pass

            # Create a new TwistStamped message
            position_error_message = TwistStamped()

            # Stamp the message
            position_error_message.header.stamp = Time.now()

            # Fill in the message fields converting from the image axes to the robot end effector axes
            position_error_message.twist.linear.x = position_error[0]

            # Publish positioning error
            self.image_based_position_error_publisher.publish(position_error_message)

        self.publish_node_status(new_status=current_status, delay_publishing=0.5, default_status=NO_IMAGES_AVAILABLE)

        # Return the results for validation
        return position_error


if __name__ == '__main__':

    # create node object
    controller = ImagePositioningControllerNode()

    # Define publishing frequency
    publishing_rate = Rate(55)  # hz

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # Let the program run indefinitely
    while not is_shutdown():
        # Publish the image based control input
        controller.publish_position_error()

        # Then wait
        publishing_rate.sleep()





