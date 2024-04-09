#!/usr/bin/env python3

"""
File containing VolumeGenerationNode class definition and ROS running code.
"""
from os import listdir, mkdir

# Import standard python packages
from numpy import array, mgrid, ones, vstack, load
from plotly.graph_objects import Figure, Mesh3d, Scatter3d
from plotly.subplots import make_subplots
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
from alphashape import alphashape, optimizealpha
from os.path import isdir, isfile
from rospy import Time
from numpy import savez
import open3d as o3d
from cv2 import contourArea

from thyroid_ultrasound_imaging_support.RegisteredData.MessageCompatibleObject import SAVE_OBJECT
# Import custom python packages
from thyroid_ultrasound_imaging_support.RegisteredData.RegisteredData import RegisteredData
from thyroid_ultrasound_imaging_support.RegisteredData.load_folder_of_saved_registered_data import \
    load_folder_of_saved_registered_data
from thyroid_ultrasound_imaging_support.Validation.date_stamp_str import date_stamp_str

# Import standard ROS packages

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import RegisteredDataMsg, NonRealTimeImageFilterStatus
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_robot_control_support.Controllers.FeatureBasedController.Surface import Surface

# TODO - Dream - Make this grow the volume incrementally
# TODO - Dream - Add proper try-catch error checking everywhere and incorporate logging into it
# TODO - Dream - Add logging through the BasicNode class

# Define volume data file prefix
VOLUME_DATA_PREFIX = 'VolumeData'

POINTS = 'points'
VECTORS = 'vectors'


class VolumeGenerationNode(BasicNode):
    def __init__(self):
        """
        Creates a VolumeGenerationNode object.
        """

        # Call the constructor of the super class
        super(VolumeGenerationNode, self).__init__()

        # Define a flag showing if the node has been commanded to create a volume
        self.commanded_to_create_volume = False

        # Define a flag showing if the node has been commanded to display a volume
        self.commanded_to_display_volume = False

        self.all_data_sent = False
        self.total_data_to_receive = 1000000000

        # Define a flag to monitor if data should be collected
        self.currently_gathering_data = False

        # Define a counter to track the number of messages of points received
        self.total_data_gathered = 0

        # Define a list to store the points used to generate the volume
        self.points_in_volume = []

        # Define a list to store the vectors used to generate the volume
        self.vectors_in_volume = []

        self.stored_registered_data_msgs = []

        self.stored_registered_data_objects = []

        self.registered_data_load_location = ''

        self.volume_data_save_location = '/home/ben/thyroid_ultrasound_data/testing_and_validation/volume_data'

        self.volume_data_load_location = ''

        self.fig = None
        #
        # plt.ion()

        # self.fig = plt.figure()
        # self.axis = self.fig.add_subplot(111, projection='3d')

        # Create the node object
        init_node(VOLUME_GENERATION)

        # Define the shutdown behaviour of the node
        on_shutdown(self.shut_down_node)

        # Publishes the volume data for the 3D volume
        self.volume_publisher = Publisher(VOLUME_DATA, Float64, queue_size=1)

        # Listens for the list of registered data
        Subscriber(REGISTERED_DATA_NON_REAL_TIME, RegisteredDataMsg, self.registered_data_callback)

        # Listens for the segmentation status of the non-real-time image filter
        Subscriber(REGISTERED_DATA_NON_REAL_TIME_SEGMENTATION_PROGRESS, NonRealTimeImageFilterStatus,
                   self.non_real_time_image_filter_progress_callback)

        # Services for data locations
        Service(VG_REGISTERED_DATA_LOAD_LOCATION, StringRequest, self.registered_data_load_location_handler)
        Service(VG_VOLUME_DATA_SAVE_LOCATION, StringRequest, self.volume_data_save_location_handler)
        Service(VG_VOLUME_DATA_LOAD_LOCATION, StringRequest, self.volume_data_load_location_handler)

        # Command services
        Service(VG_GENERATE_VOLUME, BoolRequest, self.create_volume_command_handler)
        Service(VG_DISPLAY_VOLUME, BoolRequest, self.display_volume_command_handler)

    def shut_down_node(self):
        if self.fig is not None:
            try:
                plt.close(self.fig)
            except:
                pass

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
            self.stored_registered_data_msgs.append(msg)

            # Update the number of points that have been gathered
            self.total_data_gathered = self.total_data_gathered + 1

            # Convert the message back to a RegisteredData object
            # msg_as_object = RegisteredData(source_message=msg)

            print("num received: " + str(self.total_data_gathered) + '\n')

    def create_volume_command_handler(self, req: BoolRequestRequest):
        """
        Saves the value sent in the message to the internal commanded_to_create_volume flag.
        """
        # Save the value sent in the command
        self.commanded_to_create_volume = req.value
        return BoolRequestResponse(True, NO_ERROR)

    def display_volume_command_handler(self, req: BoolRequestRequest):
        """
        Saves the value sent in the message to the internal commanded_to_display_volume flag.
        """
        # Save the value sent in the command
        self.commanded_to_display_volume = req.value

        # Load the saved data objects
        self.stored_registered_data_objects = load_folder_of_saved_registered_data(self.volume_data_load_location)

        return BoolRequestResponse(True, NO_ERROR)

    def non_real_time_image_filter_progress_callback(self, msg: NonRealTimeImageFilterStatus):
        """
        Updates
        """
        self.all_data_sent = msg.is_all_data_filtered
        self.total_data_to_receive = msg.total_images_to_filter
        print("total to receive: " + str(self.total_data_to_receive) + '\n')

    def registered_data_load_location_handler(self, req: StringRequestRequest):
        if isdir(req.value):
            self.registered_data_load_location = req.value
            return StringRequestResponse(True, NO_ERROR)

        return StringRequestResponse(False, 'Invalid directory')

    def volume_data_save_location_handler(self, req: StringRequestRequest):
        if isdir(req.value):
            self.volume_data_save_location = req.value
            return StringRequestResponse(True, NO_ERROR)

        return StringRequestResponse(False, 'Invalid directory')

    def volume_data_load_location_handler(self, req: StringRequestRequest):
        if isdir(req.value):
            self.volume_data_load_location = req.value
            return StringRequestResponse(True, NO_ERROR)

        return StringRequestResponse(False, 'Invalid directory')

    def main_loop(self):
        """
        Creates the volume and visualizes it when commanded to and all data has been received.
        """

        data_creation_successful = False
        display_data_loading_successful = False

        # If commanded to generate the volume but not display it
        if self.commanded_to_create_volume and self.all_data_sent and \
                len(self.stored_registered_data_msgs) == self.total_data_to_receive and \
                not self.commanded_to_display_volume:
            # Populate the stored objects list
            self.stored_registered_data_objects = [RegisteredData(source_message=msg) for msg in
                                                   self.stored_registered_data_msgs]

        # If the node has been commanded to visualize the data
        if (self.commanded_to_create_volume and
            len(self.stored_registered_data_objects) == self.total_data_to_receive) or \
                (self.commanded_to_display_volume and len(self.stored_registered_data_objects) ==
                 len(listdir(self.volume_data_load_location))):

            # Define lists of
            image_data_objects = []
            robot_poses = []

            # For each registered data object,
            for registered_data_object in self.stored_registered_data_objects:

                # Pull out the corresponding data
                image_data = registered_data_object.image_data
                robot_pose_at_image_capture_time = registered_data_object.robot_pose.robot_pose

                # Pull out the image data and pose
                image_data_objects.append(image_data)
                robot_poses.append(robot_pose_at_image_capture_time)

                # Generate a transformed list of points for the largest contour
                transformed_contours, transformed_vectors = image_data.generate_transformed_contours(
                    transformation=robot_pose_at_image_capture_time,
                    imaging_depth=image_data.imaging_depth / 100)

                # Add the points from each contour into the full list of points
                for contour, vector_contour in zip(transformed_contours, transformed_vectors):
                    sampling_rate = round(len(contour) / 100)
                    if sampling_rate < 1:
                        sampling_rate = 1
                    for point, vector in zip(contour[::sampling_rate], vector_contour[::sampling_rate]):
                        self.points_in_volume.append(tuple(point))
                        self.vectors_in_volume.append(tuple(vector))

            contour_areas = [contourArea(image.contours_in_image[0]) *
                             ((image.imaging_depth * .01 / image.ds_image_size_x) *
                              (image.imaging_depth * .01 / image.ds_image_size_x)) for image in image_data_objects]
            distances = [abs(Surface([robot_poses[ii][0:3, 3],
                                      robot_poses[ii][0:3, 3] + array([0, 1, 0]),
                                      robot_poses[ii][0:3, 3] + array([0, 0, 1])]).distance_to_surface(
                robot_poses[ii + 1][0:3, 3])) for ii in range(len(robot_poses) - 1)]

            volume_direction_1 = 0
            volume_direction_2 = 0

            for jj in range(0, len(contour_areas) - 1):
                volume_direction_1 = volume_direction_1 + (contour_areas[jj] * distances[jj])
                volume_direction_2 = volume_direction_2 + (contour_areas[jj + 1] * distances[jj])

            self.volume_publisher.publish(Float64(((volume_direction_1 + volume_direction_2) / 2) * 1000000))

            print("In final loop")
            # Define a temporary array for storing the points in the volume
            points_in_volume_as_array = array(self.points_in_volume)
            vectors_in_volume_as_array = array(self.vectors_in_volume)

            # Save the registered data making up the volume
            if self.commanded_to_create_volume:

                # Create a timestamp for the current time
                current_time_timestamp = date_stamp_str(prefix='_', suffix='/')

                # Name the directory where the information will be saved
                this_save_directory = (self.volume_data_save_location + '/' + VOLUME_DATA_PREFIX +
                                       self.registered_data_load_location[-len(current_time_timestamp) + 1:] +
                                       current_time_timestamp)

                # Make the directory
                mkdir(this_save_directory)

                # Save each registered data object
                for registered_data_object in self.stored_registered_data_objects:
                    registered_data_object.save_load(SAVE_OBJECT, this_save_directory)

            data_creation_successful = True

        if data_creation_successful:
            # alpha = 0.95 * optimizealpha(points_in_volume_as_array)
            # shape = alphashape(points_in_volume_as_array, alpha)

            # Build the hull representing the thyroid
            # constructed_volume = ConvexHull(points_in_volume_as_array)

            # # Publish the volume of the hull
            # self.volume_publisher.publish(Float64(constructed_volume.volume))
            #
            # Define the sampling rate to use to create the figures
            sampling_rate = 1
            #
            # # Display the volume of the hull
            # print("Thyroid Volume: " + str(constructed_volume.volume) + " m^2")
            # print("Thyroid Volume: " + str(constructed_volume.volume * 1000 ** 2) + " mm^2")
            # print("Total Points Sampled: " + str(len(self.points_in_volume)))
            # print("Points in Figure: " + str(len(points_in_volume_as_array[::sampling_rate, 0])))
            #
            fig = make_subplots(1, 1)
            fig.add_trace(Scatter3d(x=array(points_in_volume_as_array[::sampling_rate, 0]).flatten(),
                                    y=array(points_in_volume_as_array[::sampling_rate, 1]).flatten(),
                                    z=array(points_in_volume_as_array[::sampling_rate, 2]).flatten(),
                                    marker=dict(size=2,
                                                color='black')))
            fig.add_trace(Mesh3d(
                x=array(points_in_volume_as_array[::sampling_rate, 0]).flatten(),
                y=array(points_in_volume_as_array[::sampling_rate, 1]).flatten(),
                z=array(points_in_volume_as_array[::sampling_rate, 2]).flatten(),
                color='red',
                opacity=.90,
                alphahull=6.5
            ))
            # fig = Figure(data=Mesh3d(
            #     x=array(points_in_volume_as_array[::sampling_rate, 0]).flatten(),
            #     y=array(points_in_volume_as_array[::sampling_rate, 1]).flatten(),
            #     z=array(points_in_volume_as_array[::sampling_rate, 2]).flatten(),
            #     color='red',
            #     opacity=1.0,
            #     alphahull=5.5
            # ))
            fig.update_layout(title_text='Volume Rendering')
            fig.show()

            # radii = [0.005, 0.01, 0.02, 0.05]
            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(points_in_volume_as_array)
            # pcd.estimate_normals(
            #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            # # pcd.normals = o3d.utility.Vector3dVector(vectors_in_volume_as_array)
            # rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            #     pcd, o3d.utility.DoubleVector(radii))
            # o3d.visualization.draw_geometries([pcd, rec_mesh])

            # with o3d.utility.VerbosityContextManager(
            #         o3d.utility.VerbosityLevel.Debug) as cm:
            #     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            #         pcd, depth=9)
            # o3d.visualization.draw_geometries([mesh],
            #                                   zoom=0.664,
            #                                   front=[-0.4761, -0.4698, -0.7434],
            #                                   lookat=[1.8900, 3.2596, 0.9284],
            #                                   up=[0.2304, -0.8825, 0.4101])

            # Recreate the figure window if it was closed
            # self.fig = plt.figure()
            # axis = self.fig.add_subplot(111, projection='3d')
            #
            #
            # # Plot the data as a scatterplot
            # axis.scatter(points_in_volume_as_array[::sampling_rate, 0],
            #                     points_in_volume_as_array[::sampling_rate, 1],
            #                     points_in_volume_as_array[::sampling_rate, 2],
            #                     s=1)
            #
            # # Set the labels
            # axis.set_xlabel('X (m)')
            # axis.set_ylabel('Y (m)')
            # axis.set_zlabel('Z (m)')
            #
            # # Show the plot
            # plt.show()

            # Reset the flags
            self.commanded_to_create_volume = False
            self.commanded_to_display_volume = False
            self.all_data_sent = False
            self.stored_registered_data_msgs = []
            self.stored_registered_data_objects = []
            self.points_in_volume = []

            # Clear the plot
            # axis.cla()


if __name__ == '__main__':
    # create node object
    node = VolumeGenerationNode()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # While the node has not been shutdown, generate volumes when needed
    while not is_shutdown():
        node.main_loop()

    print("Node terminated.")
