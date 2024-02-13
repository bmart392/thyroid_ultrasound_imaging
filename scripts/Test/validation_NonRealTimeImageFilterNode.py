
from std_msgs.msg import String, Bool
from NonRealTimeImageFilterNode import NonRealTimeImageFilterNode


# Create the node to validate
validation_node = NonRealTimeImageFilterNode()

# Define where the test data is stored
REGISTRATION_DATA_LOCATION = '/home/ben/thyroid_ultrasound_data/testing_and_validation/registered_data'

# Send the location to the node
validation_node.registered_data_load_location_callback(String(REGISTRATION_DATA_LOCATION))

# Send the generate-volume command
validation_node.generate_volume_command_callback(Bool(True))
