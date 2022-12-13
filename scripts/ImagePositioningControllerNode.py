
from ImagePositioningController import *


if __name__ == '__main__':

    # ROS node startup stuff
    node_is_running = True

    # Set up to subscribe to the image size topic
    # Set up to subscribe to the thyroid centroid topic
    # Set up to publish to the positioning error topic

    # Create an image positioning controller for use in the node
    image_positioning_controller = ImagePositioningController()
    # While the node is running
    while node_is_running:
        # When a new centroid and image size is published
        centroids = []
        image_size = ()

        # Calculate the error in the image centroids
        positioning_error = image_positioning_controller.calculate_error_in_meters(centroids, image_size)

        # Transform the error into the necessary control input
        control_input = image_positioning_controller.calculate_control_input(positioning_error)

        # Publish positioning error
        # Publish control input



