"""
File containing the validation code for the ImagePositionRegistrationNode
"""
# Import standard python packages
from numpy import identity
from rospy import Time, Rate

# Import standard ROS packages
from std_msgs.msg import Bool

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_saved_image_data import \
    load_folder_of_saved_image_data
from thyroid_ultrasound_imaging_support.RegisteredData.MessageCompatibleObject import TO_MESSAGE
from thyroid_ultrasound_imaging_support.RegisteredData.RobotForce import RobotForce
from thyroid_ultrasound_imaging_support.RegisteredData.RobotPose import RobotPose

# Import custom ROS packages
from ImagePositionRegistrationNode import ImagePositionRegistrationNode

# Make a node
validation_node = ImagePositionRegistrationNode()

# Define the location where the image data folders are located
IMAGE_DATA_LOCATION: str = '/home/ben/thyroid_ultrasound_data/testing_and_validation/saved_image_data_objects'

# Load multiple image data objects
image_data_objects = load_folder_of_saved_image_data(IMAGE_DATA_LOCATION)

# Send a message saying pose feedback is in use
validation_node.pose_command_callback(Bool(True))

looping_rate = Rate(30)  # Hz

# For each image data object
for image_data_object in image_data_objects:

    # Capture the current time
    current_time = image_data_object.image_capture_time

    # Define that the image data object is ImageData
    image_data_object: ImageData

    # Pull out its time stamp
    # image_data_object.image_capture_time = current_time

    # Create a fake robot force with the same timestamp
    fake_robot_force = RobotForce(robot_force=[0, 0, 0, 0, 3.5, 0])
    fake_robot_force.time_stamp = current_time

    # Create a fake robot pose with the same timestamp
    fake_robot_pose = RobotPose(robot_pose=identity(4))
    fake_robot_pose.time_stamp = current_time

    # Send the messages
    validation_node.pose_callback(fake_robot_pose.convert_object_message(direction=TO_MESSAGE))
    validation_node.force_callback(fake_robot_force.convert_object_message(direction=TO_MESSAGE))

    # Send a goal reached message
    validation_node.pose_goal_callback(Bool(True))

    # Send the image data object
    validation_node.filtered_image_callback(image_data_object.convert_to_message())

    # Do the main loop
    validation_node.main_loop()
    print("Number of outer level dictionaries:" + str(len(validation_node.list_of_filtered_images.keys())))

    # Send a goal not reached message
    validation_node.pose_goal_callback(Bool(False))

    looping_rate.sleep()

