#!/usr/bin/env python3

"""
File containing VolumeGenerationNode class definition and ROS running code.
"""
from scipy.spatial import ConvexHull

# Import standard ROS packages
from rospy import init_node, Subscriber, Publisher, spin
from std_msgs.msg import Bool, Float32

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import transformed_points

# Import standard packages
from numpy import array
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Import custom packages
from thyroid_ultrasound_imaging_support.ImageData.bridge_list_of_points_multi_array import \
    bridge_list_of_points_multi_array
from thyroid_ultrasound_imaging_support.ImageData.BridgeImageDataMessageConstants import TO_OBJECT


# TODO Make this grow the volume incrementally
class VolumeGenerationNode:
    def __init__(self):
        # Create the node object
        init_node("VolumeGenerationNode")

        # Listens for the list of transformed points
        Subscriber('/image_data/transformed_points', transformed_points, self.transformed_points_callback)

        # Listens for the command to gather points
        Subscriber('/command/gather_volume_data', Bool, self.gather_volume_data_callback)

        # Listens for the command to build a 3D volume
        Subscriber('/command/create_volume', Bool, self.create_volume_callback)

        # Publishes the volume data for the 3D volume
        self.volume_publisher = Publisher('/image_data/calculated_volume', Float32, queue_size=1)

        # Define a flag to monitor if data should be collected
        self.currently_gathering_data = False

        # Define a counter to track the number of messages of points received
        self.num_points_gathered = 0

        # Define a list to store the points used to generate the volume
        self.points_in_volume = []

    def transformed_points_callback(self, data: transformed_points):

        # Check if a volume is being generated currently
        if self.currently_gathering_data:

            # Merge the new list of points with the existing list
            self.points_in_volume = self.points_in_volume + \
                                    bridge_list_of_points_multi_array(TO_OBJECT, array_message=data.transformed_points)

            # Increment the message counter
            self.num_points_gathered = self.num_points_gathered + 1

    def gather_volume_data_callback(self, data: Bool):

        # Save the value sent in the command
        self.currently_gathering_data = data.data

    def create_volume_callback(self, data: Bool):

        # If enough data has been collected so far
        if self.num_points_gathered > 1:

            # Build the hull representing the thyroid
            constructed_volume = ConvexHull(array(self.points_in_volume))

            # Publish the volume of the hull
            self.volume_publisher.publish(Float32(constructed_volume.volume))

            # Display the volume of the hull
            print("Thyroid Volume: " + str(constructed_volume.volume) + " mm^2")

            # Create the figure to visualize the volume in
            volume_figure = plt.figure()
            volume_axis = volume_figure.add_subplot(111, projection='3d')

            # Plot the data as a scatterplot
            volume_axis.scatter(self.points_in_volume[:, 0],
                                self.points_in_volume[:, 1],
                                self.points_in_volume[:, 2],
                                s=1)

            # Set the labels
            volume_axis.set_xlabel('X (mm)')
            volume_axis.set_ylabel('Y (mm)')
            volume_axis.set_zlabel('Z (mm)')

            # Show the plot
            plt.show()


if __name__ == '__main__':
    # create node object
    node = VolumeGenerationNode()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # Spin the node until it is terminated
    # Callbacks handle all functionality
    spin()
