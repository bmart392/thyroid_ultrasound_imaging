#!/usr/bin/env python3

"""
File containing ImageFiterNode class definition and ROS running code.
"""

# Import ROS packages
from rospy import init_node, spin, Service, Publisher
from thyroid_ultrasound_imaging.msg import image_data_message, image_crop_coordinates
from thyroid_ultrasound_imaging.srv import SelectCropCoordinates

# Import custom objects
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.UserInput.user_input_crop_coordinates import user_input_crop_coordinates


def request_handler(request: image_data_message):

    # Create a publisher to publish the resulting coordinates from the user image cropping
    result_publisher = Publisher('/ui/image_crop_coordinates', image_crop_coordinates, queue_size=1)

    # Create an image data object from the image_data message sent
    temp_image_data = ImageData(image_data_msg=request)

    # Capture the image crop coordinates from the image
    image_crop_coordinates_list = user_input_crop_coordinates(temp_image_data)

    # Create a message to publish the result
    result_msg = image_crop_coordinates()

    # Fill in the message with the proper data
    result_msg.first_coordinate_x = image_crop_coordinates_list[0][0]
    result_msg.first_coordinate_y = image_crop_coordinates_list[0][1]
    result_msg.second_coordinate_x = image_crop_coordinates_list[1][0]
    result_msg.second_coordinate_y = image_crop_coordinates_list[1][1]

    # Publish the response
    result_publisher.publish(result_msg)


if __name__ == '__main__':

    # Initialize the server
    init_node('select_crop_coordinates_server')
    print("Server initialized.")

    # Create service handle
    s = Service('select_crop_coordinates', SelectCropCoordinates, request_handler)

    print("Ready to serve.")

    # Wait for a request to come in
    spin()
