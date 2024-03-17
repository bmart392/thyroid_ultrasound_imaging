from VisualizationNode import VisualizationNode
from VolumeGenerationNode import VolumeGenerationNode
from std_msgs.msg import Bool, Float64
from thyroid_ultrasound_imaging_support.RegisteredData.RegisteredData import RegisteredData, TO_MESSAGE
from thyroid_ultrasound_imaging_support.RegisteredData.load_folder_of_saved_registered_data import \
    load_folder_of_saved_registered_data
from thyroid_ultrasound_imaging_support.Visualization.VisualizationConstants import IMG_CONTINUOUS
from thyroid_ultrasound_messages.msg import NonRealTimeImageFilterStatus

# Create the node to validate
validation_node = VolumeGenerationNode()

# Load the registered data
all_registered_data: list = load_folder_of_saved_registered_data('/home/ben/thyroid_ultrasound_data/experimentation/VolumeTest3/TransformedData')

# Prepare the node
validation_node.create_volume_command_handler(Bool(True))

# Send all of the data to the node
for registered_data in all_registered_data:
    registered_data: RegisteredData
    validation_node.registered_data_callback(registered_data.convert_object_message(direction=TO_MESSAGE))

validation_node.all_data_sent = True
validation_node.total_data_to_receive = len(validation_node.stored_registered_data_msgs)

# call the main loop
validation_node.main_loop()



