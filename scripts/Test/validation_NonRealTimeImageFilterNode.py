
from std_msgs.msg import String, Bool
from NonRealTimeImageFilterNode import NonRealTimeImageFilterNode


# Create the node to validate
validation_node = NonRealTimeImageFilterNode()

# Define where the test data is stored
REGISTRATION_DATA_LOCATION = '/home/ben/thyroid_ultrasound_data/experimentation/VolumeTest3/OriginalData'

# Send the location to the node
validation_node.registered_data_load_location_handler(String(REGISTRATION_DATA_LOCATION))

# Send the generate-volume command

validation_node.generate_volume_command_handler(Bool(True))

while not validation_node.commanded_to_generate_volume:
    pass

validation_node.main_loop()
