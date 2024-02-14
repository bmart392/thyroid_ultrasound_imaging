from VolumeGenerationNode import VolumeGenerationNode
from std_msgs.msg import Bool, Float64
from thyroid_ultrasound_imaging_support.RegisteredData.RegisteredData import RegisteredData, TO_MESSAGE
from thyroid_ultrasound_imaging_support.RegisteredData.load_folder_of_saved_registered_data import \
    load_folder_of_saved_registered_data
from thyroid_ultrasound_messages.msg import NonRealTimeImageFilterStatus

# Create the node to validate
validation_node = VolumeGenerationNode()

# Load the registered data
all_registered_data: list = load_folder_of_saved_registered_data('/home/ben/thyroid_ultrasound_data/'
                                                           'testing_and_validation/non_real_time_registered_data')

# Prepare the node
validation_node.create_volume_command_callback(Bool(True))
validation_node.image_depth_callback(Float64(0.05))

# Send all of the data to the node
for registered_data in all_registered_data:
    registered_data: RegisteredData
    validation_node.registered_data_callback(registered_data.convert_object_message(direction=TO_MESSAGE))
    validation_node.non_real_time_image_filter_progress_callback(
        NonRealTimeImageFilterStatus(total_images_to_filter=len(all_registered_data)))

# call the main loop
validation_node.main_loop()



