#!/usr/bin/env python3

"""
File containing VolumeGenerationNode class definition and ROS running code.
"""

# Import standard python packages
from numpy import array, mgrid, ones
from plotly.graph_objects import Figure, Mesh3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull

# Import custom python packages
from thyroid_ultrasound_imaging_support.RegisteredData.RegisteredData import RegisteredData

# Import standard ROS packages

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import RegisteredDataMsg, NonRealTimeImageFilterStatus
from thyroid_ultrasound_support.BasicNode import *


# TODO - Dream - Make this grow the volume incrementally
# TODO - Dream - Add proper try-catch error checking everywhere and incorporate logging into it
# TODO - Dream - Add logging through the BasicNode class

class VolumeGenerationNode(BasicNode):
    def __init__(self):
        """
        Creates a VolumeGenerationNode object.
        """

        # Call the constructor of the super class
        super(VolumeGenerationNode, self).__init__()

        # Define a flag showing if the node has been commanded to create a volume
        self.commanded_to_create_volume = False

        # Define a flag for noting if all the images have been received from the non-real-time image filter
        self.all_data_received = False

        # Define a flag to monitor if data should be collected
        self.currently_gathering_data = False

        # Define a counter to track the number of messages of points received
        self.num_points_gathered = 0

        # Define a list to store the points used to generate the volume
        self.points_in_volume = []

        # Define a variable to store the image depth
        self.image_depth = 0.05  # m

        # Create the node object
        init_node(VOLUME_GENERATION)

        # Publishes the volume data for the 3D volume
        self.volume_publisher = Publisher(VOLUME_DATA, Float64, queue_size=1)

        # Listens for the list of registered data
        Subscriber(REGISTERED_DATA_NON_REAL_TIME, RegisteredDataMsg, self.registered_data_callback)

        # Listens for the segmentation status of the non-real-time image filter
        Subscriber(REGISTERED_DATA_NON_REAL_TIME_SEGMENTATION_PROGRESS, NonRealTimeImageFilterStatus)

        # Listens for the command to build a 3D volume
        Subscriber(GENERATE_VOLUME, Bool, self.create_volume_command_callback)

        # Listen for the imaging depth of the image
        Subscriber(IMAGE_DEPTH, Float64, self.image_depth_callback)

    def registered_data_callback(self, msg: RegisteredDataMsg):
        """
        Saves the registered data object contained within the message after transforming the contours found in the
        image data into the origin frame.

        Parameters
        ----------
        msg :
            A message containing a registered data object which must contain an image data object and a robot pose.
        """

        # Check if a volume is being generated currently
        if self.commanded_to_create_volume:

            # Update the number of points that have been gathered
            self.num_points_gathered = self.num_points_gathered + 1

            # Convert the message back to a RegisteredData object
            msg_as_object = RegisteredData(source_message=msg)

            # Pull out the corresponding data
            image_data = msg_as_object.image_data
            robot_pose_at_image_capture_time = msg_as_object.robot_pose

            # Generate a transformed list of points for the largest contour
            transformed_contours = image_data.generate_transformed_contours(
                transformation=robot_pose_at_image_capture_time.robot_pose,
                imaging_depth=self.image_depth)

            # Add the points from each contour into the full list of points
            for contour in transformed_contours:
                for point in contour:
                    self.points_in_volume.append(point)

        else:
            # Reset the counter when the command to create the volume has been cancelled
            self.num_points_gathered = 0

            # Reset the flag that all data has been received
            self.all_data_received = False

    def create_volume_command_callback(self, msg: Bool):
        """
        Saves the value sent in the message to the internal commanded_to_create_volume flag.
        """
        # Save the value sent in the command
        self.commanded_to_create_volume = msg.data

    def image_depth_callback(self, msg: Float64):
        """
        Saves the value sent in the message to the internal image_depth field.
        """
        self.image_depth = msg.data

    def non_real_time_image_filter_progress_callback(self, msg: NonRealTimeImageFilterStatus):
        """
        Updates the internal attribute all_data_received based on how many total points should have been received.
        """
        self.all_data_received = self.num_points_gathered == msg.total_images_to_filter and msg.total_images_to_filter != 0

    def main_loop(self):
        """
        Creates the volume and visualizes it when commanded to and all data has been received.
        """

        # If the node has been commanded to generate the volume and all the data has been received
        if self.commanded_to_create_volume and self.all_data_received:
            # Define a temporary array for storing the points in the volume
            points_in_volume_as_array = array(self.points_in_volume)

            # Build the hull representing the thyroid
            constructed_volume = ConvexHull(points_in_volume_as_array)

            # Publish the volume of the hull
            self.volume_publisher.publish(Float64(constructed_volume.volume))

            # Define the sampling rate to use to create the figures
            sampling_rate = 5

            # Display the volume of the hull
            print("Thyroid Volume: " + str(constructed_volume.volume) + " m^2")
            print("Thyroid Volume: " + str(constructed_volume.volume * 1000 ** 2) + " mm^2")
            print("Total Points Sampled: " + str(len(self.points_in_volume)))
            print("Points in Figure: " + str(len(points_in_volume_as_array[::sampling_rate, 0])))

            fig = Figure(data=Mesh3d(
                x=array(points_in_volume_as_array[::sampling_rate, 0]).flatten(),
                y=array(points_in_volume_as_array[::sampling_rate, 1]).flatten(),
                z=array(points_in_volume_as_array[::sampling_rate, 2]).flatten(),
                color='red',
                opacity=1.0,
                alphahull=0
            ))
            fig.update_layout(title_text='Volume Rendering')
            fig.show()

            # # Create the figure to visualize the volume in
            # volume_figure = plt.figure()
            # volume_axis = volume_figure.add_subplot(111, projection='3d')
            #
            # # Plot the data as a scatterplot
            # volume_axis.scatter(points_in_volume_as_array[::5, 0],
            #                     points_in_volume_as_array[::5, 1],
            #                     points_in_volume_as_array[::5, 2],
            #                     s=1)
            #
            # # Set the labels
            # volume_axis.set_xlabel('X (m)')
            # volume_axis.set_ylabel('Y (m)')
            # volume_axis.set_zlabel('Z (m)')
            #
            # # Show the plot
            # plt.show()


if __name__ == '__main__':
    # create node object
    node = VolumeGenerationNode()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # While the node has not been shutdown, generate volumes when needed
    while not is_shutdown():
        node.main_loop()

    print("Node terminated.")
